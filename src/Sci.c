#include "Sci.h"

/* ============================================================
 * 环形缓冲区结构
 *
 * 这里只做“单生产者 + 单消费者”：
 *   生产者：ADC ISR
 *   消费者：main() 里的 SCI_ServiceTx()
 *
 * 所以只需要 head / tail 两个索引，不需要 count。
 *
 * buf[] 里每个元素存一个“字节”，但用 Uint16 存放，
 * 低 8 位有效，便于在 C28x 上操作。
 * ============================================================ */
typedef struct {
  volatile Uint16 head;                 /* 写入位置 */
  volatile Uint16 tail;                 /* 读取位置 */
  volatile Uint16 overflow_cnt;         /* 溢出计数 */
  volatile Uint16 buf[SCI_TX_BUF_SIZE]; /* 环形缓冲区 */
} SCI_TX_RINGBUF;

static SCI_TX_RINGBUF gSciTxBuf;

/* ============================================================
 * float 拆分联合体
 * 保留你原来“把 float 拆成 4 个字节”的方法
 * ============================================================ */
typedef union {
  float32 f;
  Uint16 w[2];
} SCI_FLOAT_UNION;

/* ============================================================
 * 内部函数声明
 * ============================================================ */
static void SCI_RB_Init(void);
static Uint16 SCI_RB_Free(void);
static Uint16 SCI_RB_PushByte(Uint16 byte_data);
static Uint16 SCI_RB_PopByte(Uint16 *byte_data);

/* ============================================================
 * 初始化环形缓冲区
 * ============================================================ */
static void SCI_RB_Init(void) {
  gSciTxBuf.head = 0U;
  gSciTxBuf.tail = 0U;
  gSciTxBuf.overflow_cnt = 0U;
}

/* ============================================================
 * 计算缓冲区当前已用字节数
 * ============================================================ */
static Uint16 SCI_RB_Used(void) {
  return (Uint16)((gSciTxBuf.head - gSciTxBuf.tail) & SCI_TX_BUF_MASK);
}

/* ============================================================
 * 计算缓冲区剩余空间
 * 说明：
 *   这里保留 1 个空位区分“满”和“空”
 * ============================================================ */
static Uint16 SCI_RB_Free(void) {
  return (Uint16)(SCI_TX_BUF_MASK - SCI_RB_Used());
}

/* ============================================================
 * 向缓冲区写入 1 个字节---入队
 * 返回：
 *   1 = 成功
 *   0 = 缓冲区满
 * ============================================================ */
static Uint16 SCI_RB_PushByte(Uint16 byte_data) {
  Uint16 next_head;

  byte_data &= 0x00FFU; /* 只保留低 8 位 */

  next_head = (Uint16)((gSciTxBuf.head + 1U) & SCI_TX_BUF_MASK);
  // SCI_TX_BUF_MASK=1023=0X 11 1111 1111,1024 & 1023 = 0
  // 当head超出缓存区边界，回到0（环形缓冲）

  /* next_head == tail 说明满了 */
  if (next_head == gSciTxBuf.tail) {
    gSciTxBuf.overflow_cnt++;
    return 0U;
  }

  gSciTxBuf.buf[gSciTxBuf.head] = byte_data;
  gSciTxBuf.head = next_head;

  return 1U;
}

/* ============================================================
 * 从缓冲区读出 1 个字节---出队
 * 返回：
 *   1 = 成功
 *   0 = 缓冲区空
 * ============================================================ */
static Uint16 SCI_RB_PopByte(Uint16 *byte_data) {
  if (gSciTxBuf.head == gSciTxBuf.tail) {
    return 0U;
  }

  *byte_data = gSciTxBuf.buf[gSciTxBuf.tail] & 0x00FFU; // 只出队低八位数据
  gSciTxBuf.tail = (Uint16)((gSciTxBuf.tail + 1U) & SCI_TX_BUF_MASK);

  return 1U;
}

/* ============================================================
 * SCI-A 初始化
 * ============================================================ */

void SCI_Init(Uint32 baud) {
  Uint16 Scibaud;
  Uint16 Scihbaud, Scilbaud;

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
  EDIS;

  // 配置GPIO（完全复制TI官方例程）
  EALLOW;
  InitSciaGpio();
  EDIS;

  // 完全复位SCI模块
  SciaRegs.SCICTL1.all = 0x0000;
  DELAY_US(10);

  // 配置基本参数：1停止位，无校验，8位数据，空闲线模式
  SciaRegs.SCICCR.all = 0x0007;
  SciaRegs.SCICTL2.all = 0x0000; // 关闭所有中断

  // TI官方公式计算波特率
  Scibaud = (37500000UL) / (8 * baud) - 1;
  Scihbaud = Scibaud >> 8;
  Scilbaud = Scibaud & 0xFF;

  SciaRegs.SCIHBAUD = Scihbaud;
  SciaRegs.SCILBAUD = Scilbaud;

  // 配置FIFO（复制TI官方例程）
  SciaRegs.SCIFFTX.all = 0xE040; // 使能TX FIFO，清空FIFO，禁止中断
  SciaRegs.SCIFFRX.all = 0x204F; // 使能RX FIFO，清空FIFO，禁止中断
  SciaRegs.SCIFFCT.all = 0x0000;

  // 使能发送，关闭接收
  SciaRegs.SCICTL1.bit.TXENA = 1;
  SciaRegs.SCICTL1.bit.RXENA = 0;

  // 最后退出复位，SCI开始工作
  SciaRegs.SCICTL1.bit.SWRESET = 1;

  // 等待发送器完全就绪
  while ((SciaRegs.SCICTL2.all & 0x0040) == 0)
    ;

  // /* 初始化软件缓冲区 */
  SCI_RB_Init();
}

/* ============================================================
 * 这里保留你的“原始思路”：
 * 把一个 float 拆成 4 个字节，按小端顺序入队
 *
 * 注意：
 * 这里不是直接发串口，而是写到环形缓冲区里
 * ============================================================ */
static Uint16 SCI_QueueFloatToBuf(const float32 f) {
  SCI_FLOAT_UNION tx;

  tx.f = f;

  /* 低字节优先：
   * tx.w[0] 低8位
   * tx.w[0] 高8位
   * tx.w[1] 低8位
   * tx.w[1] 高8位
   */
  if (!SCI_RB_PushByte(tx.w[0] & 0x00FFU))
    return 0U;
  if (!SCI_RB_PushByte((tx.w[0] >> 8) & 0x00FFU))
    return 0U;
  if (!SCI_RB_PushByte(tx.w[1] & 0x00FFU))
    return 0U;
  if (!SCI_RB_PushByte((tx.w[1] >> 8) & 0x00FFU))
    return 0U;

  return 1U;
}

/* ============================================================
 * JustFloat 入队
 *
 * 帧格式：
 *   [float1 4字节][float2 4字节]...[floatN 4字节][00 00 80 7F]
 *
 * 返回：
 *   1 = 整帧成功入队
 *   0 = 空间不足，整帧丢弃
 *
 * 说明：
 *   这里采用“整帧入队”策略，避免只写进去半帧导致 VOFA 解帧错乱。
 * ============================================================ */
Uint16 SendFloatArray_JustFloat(const float32 *data_arr, Uint16 col_num) {
  Uint16 i;
  Uint16 needed_bytes;

  /* 每个 float 4 字节，尾帧 4 字节 */
  needed_bytes = (Uint16)(col_num * 4U + 4U);

  /* 空间不够，整帧丢弃 */
  if (SCI_RB_Free() < needed_bytes) {
    gSciTxBuf.overflow_cnt++;
    return 0U;
  }

  /* 逐个 float 入队 */
  for (i = 0U; i < col_num; i++) {
    if (!SCI_QueueFloatToBuf(data_arr[i])) {
      return 0U;
    }
  }

  /* JustFloat 尾帧 */
  if (!SCI_RB_PushByte(JUSTFLOAT_TAIL1))
    return 0U;
  if (!SCI_RB_PushByte(JUSTFLOAT_TAIL2))
    return 0U;
  if (!SCI_RB_PushByte(JUSTFLOAT_TAIL3))
    return 0U;
  if (!SCI_RB_PushByte(JUSTFLOAT_TAIL4))
    return 0U;

  return 1U;
}

/* ============================================================
 * 主循环里调用：
 * 把软件缓冲区中的字节尽量灌入 SCI TX FIFO
 *
 * 这里不等待，不阻塞。
 * FIFO 满了就退出，下一次 main() 再继续发。
 * ============================================================ */
void SCI_ServiceTx(void) {
  Uint16 fifo_free;
  Uint16 data;

  /* SCI TX FIFO 当前硬件 FIFO 还剩多少个空闲位置 */
  fifo_free = (Uint16)(16U - SciaRegs.SCIFFTX.bit.TXFFST);

  while ((fifo_free > 0U) && SCI_RB_PopByte(&data)) {
    SciaRegs.SCITXBUF = data;
    fifo_free--;
  }
}

/* ============================================================
 * 获取溢出计数
 * ============================================================ */
Uint16 SCI_GetOverflowCount(void) { return gSciTxBuf.overflow_cnt; }