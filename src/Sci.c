#include "Sci.h"

/* ============================================================
 * ���λ������ṹ
 *
 * ����ֻ������������ + �������ߡ���
 *   �����ߣ�ADC ISR
 *   �����ߣ�main() ��� SCI_ServiceTx()
 *
 * ����ֻ��Ҫ head / tail ��������������Ҫ count��
 *
 * buf[] ��ÿ��Ԫ�ش�һ�����ֽڡ������� Uint16 ��ţ�
 * �� 8 λ��Ч�������� C28x �ϲ�����
 * ============================================================ */
typedef struct {
  volatile Uint16 head;                 /* д��λ�� */
  volatile Uint16 tail;                 /* ��ȡλ�� */
  volatile Uint16 overflow_cnt;         /* ������� */
  volatile Uint16 buf[SCI_TX_BUF_SIZE]; /* ���λ����� */
} SCI_TX_RINGBUF;

static SCI_TX_RINGBUF gSciTxBuf;

/* ============================================================
 * float ���������
 * ������ԭ������ float ��� 4 ���ֽڡ��ķ���
 * ============================================================ */
typedef union {
  float32 f;
  Uint16 w[2];
} SCI_FLOAT_UNION;

/* ============================================================
 * �ڲ���������
 * ============================================================ */
static void SCI_RB_Init(void);
static Uint16 SCI_RB_Free(void);
static Uint16 SCI_RB_PushByte(Uint16 byte_data);
static Uint16 SCI_RB_PopByte(Uint16 *byte_data);

/* ============================================================
 * ��ʼ�����λ�����
 * ============================================================ */
static void SCI_RB_Init(void) {
  gSciTxBuf.head = 0U;
  gSciTxBuf.tail = 0U;
  gSciTxBuf.overflow_cnt = 0U;
}

/* ============================================================
 * ���㻺������ǰ�����ֽ���
 * ============================================================ */
static Uint16 SCI_RB_Used(void) {
  return (Uint16)((gSciTxBuf.head - gSciTxBuf.tail) & SCI_TX_BUF_MASK);
}

/* ============================================================
 * ���㻺����ʣ��ռ�
 * ˵����
 *   ���ﱣ�� 1 ����λ���֡������͡��ա�
 * ============================================================ */
static Uint16 SCI_RB_Free(void) {
  return (Uint16)(SCI_TX_BUF_MASK - SCI_RB_Used());
}

/* ============================================================
 * �򻺳���д�� 1 ���ֽ�---���
 * ���أ�
 *   1 = �ɹ�
 *   0 = ��������
 * ============================================================ */
static Uint16 SCI_RB_PushByte(Uint16 byte_data) {
  Uint16 next_head;

  byte_data &= 0x00FFU; /* ֻ������ 8 λ */

  next_head = (Uint16)((gSciTxBuf.head + 1U) & SCI_TX_BUF_MASK);
  // SCI_TX_BUF_MASK=1023=0X 11 1111 1111,1024 & 1023 = 0
  // ��head�����������߽磬�ص�0�����λ��壩

  /* next_head == tail ˵������ */
  if (next_head == gSciTxBuf.tail) {
    gSciTxBuf.overflow_cnt++;
    return 0U;
  }

  gSciTxBuf.buf[gSciTxBuf.head] = byte_data;
  gSciTxBuf.head = next_head;

  return 1U;
}

/* ============================================================
 * �ӻ��������� 1 ���ֽ�---����
 * ���أ�
 *   1 = �ɹ�
 *   0 = ��������
 * ============================================================ */
static Uint16 SCI_RB_PopByte(Uint16 *byte_data) {
  if (gSciTxBuf.head == gSciTxBuf.tail) {
    return 0U;
  }

  *byte_data = gSciTxBuf.buf[gSciTxBuf.tail] & 0x00FFU; // ֻ���ӵͰ�λ����
  gSciTxBuf.tail = (Uint16)((gSciTxBuf.tail + 1U) & SCI_TX_BUF_MASK);

  return 1U;
}

/* ============================================================
 * SCI-A ��ʼ��
 * ============================================================ */

void SCI_Init(Uint32 baud) {
  Uint16 Scibaud;
  Uint16 Scihbaud, Scilbaud;

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
  EDIS;

  // ����GPIO����ȫ����TI�ٷ����̣�
  EALLOW;
  InitSciaGpio();
  EDIS;

  // ��ȫ��λSCIģ��
  SciaRegs.SCICTL1.all = 0x0000;
  DELAY_US(10);

  // ���û���������1ֹͣλ����У�飬8λ���ݣ�������ģʽ
  SciaRegs.SCICCR.all = 0x0007;
  SciaRegs.SCICTL2.all = 0x0000; // �ر������ж�

  // TI�ٷ���ʽ���㲨����
  Scibaud = (37500000UL) / (8 * baud) - 1;
  Scihbaud = Scibaud >> 8;
  Scilbaud = Scibaud & 0xFF;

  SciaRegs.SCIHBAUD = Scihbaud;
  SciaRegs.SCILBAUD = Scilbaud;

  // ����FIFO������TI�ٷ����̣�
  SciaRegs.SCIFFTX.all = 0xE040; // ʹ��TX FIFO�����FIFO����ֹ�ж�
  SciaRegs.SCIFFRX.all = 0x204F; // ʹ��RX FIFO�����FIFO����ֹ�ж�
  SciaRegs.SCIFFCT.all = 0x0000;

  // ʹ�ܷ��ͣ��رս���
  SciaRegs.SCICTL1.bit.TXENA = 1;
  SciaRegs.SCICTL1.bit.RXENA = 0;

  // ����˳���λ��SCI��ʼ����
  SciaRegs.SCICTL1.bit.SWRESET = 1;

  // �ȴ���������ȫ����
  while ((SciaRegs.SCICTL2.all & 0x0040) == 0)
    ;

  // /* ��ʼ������������ */
  SCI_RB_Init();
}

/* ============================================================
 * ���ﱣ����ġ�ԭʼ˼·����
 * ��һ�� float ��� 4 ���ֽڣ���С��˳�����
 *
 * ע�⣺
 * ���ﲻ��ֱ�ӷ����ڣ�����д�����λ�������
 * ============================================================ */
static Uint16 SCI_QueueFloatToBuf(const float32 f) {
  SCI_FLOAT_UNION tx;

  tx.f = f;

  /* ���ֽ����ȣ�
   * tx.w[0] ��8λ
   * tx.w[0] ��8λ
   * tx.w[1] ��8λ
   * tx.w[1] ��8λ
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
 * JustFloat ���
 *
 * ֡��ʽ��
 *   [float1 4�ֽ�][float2 4�ֽ�]...[floatN 4�ֽ�][00 00 80 7F]
 *
 * ���أ�
 *   1 = ��֡�ɹ����
 *   0 = �ռ䲻�㣬��֡����
 *
 * ˵����
 *   ������á���֡��ӡ����ԣ�����ֻд��ȥ��֡���� VOFA ��֡���ҡ�
 * ============================================================ */
Uint16 SendFloatArray_JustFloat(const float32 *data_arr, Uint16 col_num) {
  Uint16 i;
  Uint16 needed_bytes;

  /* ÿ�� float 4 �ֽڣ�β֡ 4 �ֽ� */
  needed_bytes = (Uint16)(col_num * 4U + 4U);

  /* �ռ䲻������֡���� */
  if (SCI_RB_Free() < needed_bytes) {
    gSciTxBuf.overflow_cnt++;
    return 0U;
  }

  /* ��� float ��� */
  for (i = 0U; i < col_num; i++) {
    if (!SCI_QueueFloatToBuf(data_arr[i])) {
      return 0U;
    }
  }

  /* JustFloat β֡ */
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
 * ��ѭ������ã�
 * �������������е��ֽھ������� SCI TX FIFO
 *
 * ���ﲻ�ȴ�����������
 * FIFO ���˾��˳�����һ�� main() �ټ�������
 * ============================================================ */
void SCI_ServiceTx(void) {
  Uint16 fifo_free;
  Uint16 data;

  /* SCI TX FIFO ��ǰӲ�� FIFO ��ʣ���ٸ�����λ�� */
  fifo_free = (Uint16)(16U - SciaRegs.SCIFFTX.bit.TXFFST);

  while ((fifo_free > 0U) && SCI_RB_PopByte(&data)) {
    SciaRegs.SCITXBUF = data;
    fifo_free--;
  }
}

/* ============================================================
 * ��ȡ�������
 * ============================================================ */
Uint16 SCI_GetOverflowCount(void) { return gSciTxBuf.overflow_cnt; }

