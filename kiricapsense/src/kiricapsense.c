/*
 * Kirisaki Capsense
 *
 * This code is originally derived from the silabs capsense library but adapted so that the timing loop is external
 */

#include "em_device.h"
#include "em_acmp.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "kiricapsense.h"

/* Defines */
#define KCS_BUF0_IDX_MID        (KCS_BUF0_SZ / 2)
#define KCS_BUF1_IDX_MID        (KCS_BUF1_SZ / 2)
#define KCS_BUF2_IDX_MID		(KCS_BUF2_SZ / 2)
#define KCS_CBUF_MASK ((1u << KCS_CBUF_BITS) - 1u)

/** The current channel we are sensing. */
static volatile uint8_t currentChannel;

static volatile uint16_t cirBuf[KCS_NUM_CHANNELS][1u << KCS_CBUF_BITS] = { 0 };
static volatile uint8_t kcs_cbuf_head[KCS_NUM_CHANNELS] = { 0 };
static volatile uint8_t kcs_cbuf_tail[KCS_NUM_CHANNELS] = { 0 };

#if defined(KCS_CHECK_BUFFER_SZ) && KCS_CHECK_BUFFER_SZ
static volatile uint8_t kcs_cbuf_sz[KCS_NUM_CHANNELS] = { 0 };
static volatile uint8_t kcs_cbuf_sz_max[KCS_NUM_CHANNELS] = { 0 };
#endif

static uint16_t channelBaseline[KCS_NUM_CHANNELS] = { 0 };
static uint16_t baseMHDCount[KCS_NUM_CHANNELS] = { 0 };

static uint8_t buf0Samples[KCS_NUM_CHANNELS] = { 0 };
static uint8_t buf1Samples[KCS_NUM_CHANNELS] = { 0 };
static uint8_t buf2Samples[KCS_NUM_CHANNELS] = { 0 };

static uint16_t chanBuf0[KCS_NUM_CHANNELS][KCS_BUF0_SZ] = { 0 };
static uint16_t chanBuf1[KCS_NUM_CHANNELS][KCS_BUF1_SZ] = { 0 };
static uint16_t chanBuf2[KCS_NUM_CHANNELS][KCS_BUF2_SZ] = { 0 };

static const uint8_t channel2hw[] = KCS_CHANNEL_IDX_2_HW;

static volatile uint8_t chanPressAvail = 0;
static volatile uint8_t chanPress = 0;

/* Internal Function Prototypes */
static void _kcs_buf0_handle (uint8_t chan, uint16_t value);
static void _kcs_buf1_handle (uint8_t chan);
static void _kcs_buf2_handle (uint8_t chan);
inline static void _kcs_insertion_sort_0(uint8_t chan);
static void _kcs_insertion_sort_1 (uint8_t chan);
static void _kcs_insertion_sort_2 (uint8_t chan);
static void _kcs_baseline_calculation_algorithm (uint8_t chan);
uint8_t _kcs_calcPressed(uint8_t channel);

/* Insternal Function Definitions */

/* TODO: see if this actually gets optimized down to a few comparisons */
inline static void _kcs_insertion_sort_0(uint8_t chan)
{
	/* Called step here to honor shellsort
	 *
	 * Starts with sorting two elements in the array first and goes from there.
	 */
	for (uint8_t i = 1; i < KCS_BUF0_SZ; i++)
	{
		uint16_t key = chanBuf0[chan][i];
		uint8_t j = i - 1u;

		/* Compare the key to the value on the left until the array is sorted
		 *
		 * Indexing with J relies on a dirty unsigned indexing trick where
		 * overflow is exploited.
		 */
		while (j < KCS_BUF0_SZ && key < chanBuf0[chan][j])
		{
			chanBuf0[chan][j + 1u] = chanBuf0[chan][j];
			--j;
		}
		/* The loop above always ends with j indexed one above the target
		 * index where we would want to store the key.
		 *
		 * We extend the same dirty unsigned indexing trick here.
		 */
		chanBuf0[chan][j + 1u] = key;
	}
}

static void _kcs_insertion_sort_1 (uint8_t chan)
{
	/* No comments here since this is the same as the preceding function */
	for (uint8_t i = 1; i < KCS_BUF1_SZ; i++)
	{
		uint16_t key = chanBuf1[chan][i];
		uint8_t j = i - 1u;

		while (j < KCS_BUF1_SZ && key < chanBuf1[chan][j])
		{
			chanBuf1[chan][j + 1u] = chanBuf1[chan][j];
			--j;
		}
		chanBuf1[chan][j + 1u] = key;
	}
}

static void _kcs_insertion_sort_2 (uint8_t chan)
{
	/* No comments here since this is the same as the preceding function */
	for (uint8_t i = 1; i < KCS_BUF2_SZ; i++)
	{
		uint16_t key = chanBuf2[chan][i];
		uint8_t j = i - 1u;

		while (j < KCS_BUF2_SZ && key < chanBuf2[chan][j])
		{
			chanBuf2[chan][j + 1u] = chanBuf2[chan][j];
			--j;
		}
		chanBuf2[chan][j + 1u] = key;
	}
}

uint8_t _kcs_calcPressed(uint8_t channel)
{
	  uint16_t treshold;
	  /* Threshold is set to 12.5% below the maximum value */
	  /* This calculation is performed in two steps because channelBaseline is
	   * volatile. */
	  treshold  = channelBaseline[channel];
	  treshold -= channelBaseline[channel] >> 3u;

	  if (chanBuf1[channel][KCS_BUF1_IDX_MID] < treshold) {
	    return 1;
	  }
	  return 0;
}

static void _kcs_buf0_handle (uint8_t chan, uint16_t value)
{
	chanBuf0[chan][buf0Samples[chan]++] = value;
	if (buf0Samples[chan] >= KCS_BUF0_SZ)
	{
		buf0Samples[chan] = 0;
		_kcs_insertion_sort_0(chan);
		_kcs_buf1_handle(chan);
	}
}

static void _kcs_buf1_handle (uint8_t chan)
{
	chanBuf1[chan][buf1Samples[chan]++] = chanBuf0[chan][KCS_BUF0_IDX_MID];
	if (buf1Samples[chan] >= KCS_BUF1_SZ)
	{
		buf1Samples[chan] = 0;
		_kcs_insertion_sort_1(chan);
		chanPress = _kcs_calcPressed(chan) << chan | (~(1 << chan) & chanPress);
		chanPressAvail |= 1 << chan;
		_kcs_buf2_handle(chan);
	}
}

static void _kcs_buf2_handle (uint8_t chan)
{
	/* part of the median filter*/
	chanBuf2[chan][buf2Samples[chan]++] = chanBuf1[chan][KCS_BUF1_IDX_MID];
	_kcs_baseline_calculation_algorithm(chan);
}

static void _kcs_baseline_calculation_algorithm (uint8_t chan)
{
	/* Thank you to MPR121 for actually sharing a baseline algorithm */
	// case 4 is handled by the previous filter
	// check sample direction for case 2 / case 3
	if (buf2Samples[chan] > 1u)
	{
		uint16_t key = chanBuf2[chan][buf2Samples[chan]-1u];
		// check direction for mismatch.
		// the samples already in the buffer should all be the same direction
		if ((key > channelBaseline[chan]
				&& chanBuf2[chan][buf2Samples[chan]-2u] < channelBaseline[chan])
				|| (key < channelBaseline[chan]
				&& chanBuf2[chan][buf2Samples[chan]-2u] > channelBaseline[chan]))
		{
			buf2Samples[chan] = 0;
		}
	}

	if (buf2Samples[chan] >  0)
	{
		uint16_t key = chanBuf2[chan][buf2Samples[chan]-1u];

		// MHD amplitude mismatch, case 1
		// filtering for case 5 handled here
		// first comparison for overflow prevention
		if (channelBaseline[chan] > 2 * KCS_MHD && key < channelBaseline[chan] - 2 * KCS_MHD)
		{
			if (baseMHDCount[chan] < KCS_RECAL_THR)
			{
				baseMHDCount[chan]++;
				buf2Samples[chan] = 0;
			}
		}
	}

	// maximum number of samples
	if (buf2Samples[chan] >= KCS_BUF2_SZ)
	{
		uint16_t newBase = 0;
		_kcs_insertion_sort_2(chan);
		newBase = chanBuf2[chan][KCS_BUF2_IDX_MID];
		// called NHD in the application note, different here
		// could be one line of ternary, but nobody likes ternary operations
		if (newBase < channelBaseline[chan])
		{
			if (KCS_SLR_D > 0 && newBase < channelBaseline[chan] - KCS_SLR_D && baseMHDCount[chan] < KCS_RECAL_THR)
			{
				channelBaseline[chan] -= KCS_SLR_D;
			}
			else
			{
				channelBaseline[chan] = newBase;
			}
		}
		else if (newBase > channelBaseline[chan])
		{
			if (KCS_SLR_U > 0 && newBase > channelBaseline[chan] + KCS_SLR_U)
			{
				channelBaseline[chan] += KCS_SLR_U;
			}
			else
			{
				channelBaseline[chan] = newBase;
			}
		}

		buf2Samples[chan] = 0;
		baseMHDCount[chan] = 0;
	}
}

void _kcs_cirBuf_push (uint8_t chan, uint16_t val)
{
	kcs_cbuf_head[chan] &= KCS_CBUF_MASK;
	kcs_cbuf_tail[chan] &= KCS_CBUF_MASK;
	uint8_t new_head_value = (kcs_cbuf_head[chan] + 1u) & KCS_CBUF_MASK;
	/* Move the tail and discard a value if the buffer is already full */
	if (new_head_value == kcs_cbuf_tail[chan])
	{
		kcs_cbuf_tail[chan] = (kcs_cbuf_tail[chan] + 1u) & KCS_CBUF_MASK;
	}
#if defined(KCS_CHECK_BUFFER_SZ) && KCS_CHECK_BUFFER_SZ
	else
	{
		kcs_cbuf_sz[chan]++;
		if (kcs_cbuf_sz[chan] > kcs_cbuf_sz_max[chan])
		{
			kcs_cbuf_sz_max[chan] = kcs_cbuf_sz[chan];
		}
	}
#endif
	cirBuf[chan][new_head_value] = val;
	kcs_cbuf_head[chan] = new_head_value;
}

bool _kcs_cirBuf_pop (uint8_t chan, uint16_t *val)
{
	kcs_cbuf_head[chan] &= KCS_CBUF_MASK;
	kcs_cbuf_tail[chan] &= KCS_CBUF_MASK;
	/* return false if the buffer is empty */
	if (kcs_cbuf_tail[chan] == kcs_cbuf_head[chan])
		return false;
	*val = cirBuf[chan][kcs_cbuf_tail[chan]];
	kcs_cbuf_tail[chan] = (kcs_cbuf_tail[chan] + 1u) & KCS_CBUF_MASK;
#if defined(KCS_CHECK_BUFFER_SZ) && KCS_CHECK_BUFFER_SZ
	kcs_cbuf_sz[chan]--;
#endif
	return true;
}

void KIRICAPSENSE_Init(void)
{
  /* Use kirisaki's capacitive sensing setup */
  ACMP_CapsenseInit_TypeDef capsenseInit = {
    false,            /* fullBias */
    false,            /* halfBias */
    0x7,              /* biasProg */
    acmpWarmTime512,  /* 512 cycle warmup to be safe */
    acmpHysteresisLevel1,
    acmpResistor0,
    false,            /* low power reference */
    47,             /* VDD level */
    true              /* Enable after init. */
  };

  /* Enable TIMER0, TIMER1, ACMP_CAPSENSE and PRS clock */
  CMU_ClockEnable(cmuClock_TIMER1, true);
#if defined(KCS_ACMP_CAPSENSE_CMUCLOCK)
  CMU_ClockEnable(KCS_ACMP_CAPSENSE_CMUCLOCK, true);
#else
  CMU->HFPERCLKEN0 |= KCS_ACMP_CAPSENSE_CLKEN;
#endif
  CMU_ClockEnable(cmuClock_PRS, true);

  /* Initialize TIMER1 - Prescaler value does not matter, clock source CC1, top value 0xFFFF */
  TIMER1->CTRL = TIMER_CTRL_PRESC_DIV1024 | TIMER_CTRL_CLKSEL_CC1;
  TIMER1->TOP  = 0xFFFF;

  /*Set up TIMER1 CC1 to trigger on PRS channel 0 */
  TIMER1->CC[1].CTRL = TIMER_CC_CTRL_MODE_INPUTCAPTURE /* Input capture      */
                       | TIMER_CC_CTRL_PRSSEL_PRSCH0   /* PRS channel 0      */
                       | TIMER_CC_CTRL_INSEL_PRS       /* PRS input selected */
                       | TIMER_CC_CTRL_ICEVCTRL_RISING /* PRS on rising edge */
                       | TIMER_CC_CTRL_ICEDGE_BOTH;    /* PRS on rising edge */

  /*Set up PRS channel 0 to trigger on ACMP1 output*/
  PRS->CH[0].CTRL = PRS_CH_CTRL_EDSEL_POSEDGE      /* Posedge triggers action */
                    | PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE      /* PRS source */
                    | PRS_CH_CTRL_SIGSEL_ACMPOUT_CAPSENSE;     /* PRS source */

  /* Set up ACMP1 in capsense mode */
  ACMP_CapsenseInit(KCS_ACMP_CAPSENSE, &capsenseInit);
  ACMP_Enable(KCS_ACMP_CAPSENSE);
  ACMP_CapsenseChannelSet(KCS_ACMP_CAPSENSE, channel2hw[currentChannel]);
}


/* TIMER0 IRQHandler moved somewhere else. */

void KIRICAPSENSE_IT(void)
{
	uint16_t count;

	/* Stop TIMER1 */
	TIMER1->CMD = TIMER_CMD_STOP;

	/* Read out value of TIMER1 */
	count = TIMER1->CNT;
	TIMER1->CNT = 0;

	/* Restart TIMER1 */
	TIMER1->CMD = TIMER_CMD_START;

	/* Store value in circular buffer */
	_kcs_cirBuf_push(currentChannel, count);

	/* Find the next capsense channel */
	if (++currentChannel >= KCS_NUM_CHANNELS)
	{
		currentChannel = 0;
	}

	/* Set up this channel in the ACMP. */
	ACMP_CapsenseChannelSet(KCS_ACMP_CAPSENSE, channel2hw[currentChannel]);
}

void KIRICAPSENSE_process(void)
{
	for (uint8_t cc = 0; cc < KCS_NUM_CHANNELS; cc++)
	{
		uint16_t value = 0;
		while (_kcs_cirBuf_pop(cc, &value))
		{
			_kcs_buf0_handle(cc, value);
		}
	}
}

bool KIRICAPSENSE_getPressed(uint8_t channel)
{
	EFM_ASSERT(channel < KCS_NUM_CHANNELS);
	return chanPress & 1 << channel;
}

uint8_t KIRICAPSENSE_pressReady(void)
{
	if (chanPressAvail != 0)
	{
		for (uint8_t chan = 0; chan <= 7; chan++)
		{
			if (chanPressAvail & 1 << chan)
			{
				chanPressAvail &= ~(1 << chan);
				return chan;
			}
		}
	}
	return 255;
}
