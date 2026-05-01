/*
 * Copyright 2009 by Avnera Corporation, Beaverton, Oregon.
 *
 *
 * All Rights Reserved
 *
 *
 * This file may not be modified, copied, or distributed in part or in whole
 * without prior written consent from Avnera Corporation.
 *
 *
 * AVNERA DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING
 * ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO EVENT SHALL
 * AVNERA BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR
 * ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
 * WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
 * ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
 * SOFTWARE.
 *
 * \file app.c - Functions for application specific run time control.
 */

#include "types.h"
#include "apputil.h"
#include "regdefs.h"
#include "config.h"
#include "fileIds.h"
#include "timer.h"
#include "int.h"

#include "maccfg.h"
#include "appMac.h"
#include "mac.h"
#include "macaudio.h"
#include "maccmd.h"
#include "machdr.h"
#include "maccqLT.h"

#if defined(MFCFG_HOST_REGTORY)
#include "macconfig_regulatory.h"
#elif defined(MFCFG_HOST_REGTORY_FHSS)
#include "macconfig_regulatory_fhss.h"
#elif defined(MFCFG_HOST_REGTORY_DSBEU)
#include "macconfig_regulatory_dsbeu.h"
#else
#include "macconfig.h"
#endif

#include "appcmn.h"
#include "samsub.h"
#include "hSamSub.h"
#if defined(CONFIG_INCLUDE_GPIO_LOOPBACK) && (CONFIG_INCLUDE_GPIO_LOOPBACK != 0)
#include "gpioLoopback.h"
#endif // CONFIG_INCLUDE_GPIO_LOOPBACK
#include "stwis.h"
#include "twis.h"
#include "updateImage.h"
#include "serMem.h"
#include "flash.h"
#include "swa11Version.h"
#include "memutil.h"
#include "cGpio.h"
#include "debug.h"
#include "flashProtect.h"

/**********************************************************************
 * Constants
 **********************************************************************/

enum {
    HSAMSUB_MUTE_GPIO = 11,
    HSAMSUB_IRQ_GPIO = 17,
    HSAMSUB_IRQ_BIT = (1 << (HSAMSUB_IRQ_GPIO & 7)),
};

#define PRIM_AUTO_PAIRING_TIMEOUT_SECS      600
#define MANUAL_PAIRING_TIMEOUT_SECS         60
#define SECOND_AUTO_PAIRING_TIMEOUT_SECS    5

#define MacHopBand4A_SKU_C_IND  (MacHopBand4A_SKU_C + 1)
#define MacHopBand4C_SKU_E_IND  (MacHopBand4C_SKU_E + 1)
#define MacHopBand4C_SKU_F_IND  (MacHopBand4C_SKU_F + 1)
#define MacHopBand4C_SKU_G_IND  (MacHopBand4C_SKU_G + 1)

/**********************************************************************
 * ABSL - Automatic Band Selection Logic (power-on only)
 *
 * At power-on, if register 0x28 is non-zero and not 0xFF, the arbiter
 * scans every channel on the Lo band, then every channel on the Hi band,
 * counts clean channels in each, and selects the band with more clean
 * channels.  This completes before search/pair begins, so clients never
 * see a band change — fully backward compatible.
 **********************************************************************/

typedef enum {
    ABSL_IDLE,           /* ABSL not active (disabled or done) */
    ABSL_SCAN_LO,       /* Scanning Lo band, waiting for metrics */
    ABSL_SCAN_HI,       /* Scanning Hi band, waiting for metrics */
    ABSL_DONE            /* Band chosen, normal operation */
} AbslState;

static AbslState abslState = ABSL_IDLE;
static u8 abslLoCleanChans;
static MacHopBand abslLoSku = MacHopBandInvalid;
static MacHopBand abslHiSku = MacHopBandInvalid;

/**********************************************************************
 * Data Types
 **********************************************************************/

typedef enum {
    SAMSUB_OTASI_SUB = 1,
    SAMSUB_OTASI_LREAR = 2,
    SAMSUB_OTASI_RREAR = 3,
    SAMSUB_OTASI_REAR = 4,
    SAMSUB_OTASI_ALL = 5
} SamSub_OtasiTgt;

typedef struct {
    u8 twiReg;
    u8 val;
    u8 state;
    SamSub_OtasiTgt tgt;
} SamSub_CliReg;

typedef enum {
    SAMSUB_REGSTATE_DIRTY,
    SAMSUB_REGSTATE_UNSET,
    SAMSUB_REGSTATE_CLEAN,
} SamSub_RegState;

typedef enum {
    SamSubTxType_SingleTx, ///< Single transmitter, e.g. 800A, 0x40 I2C device
    SamSubTxType_DualTx1, ///< Dual transmitter 1, e.g. 950A, 0x40 I2C device
    SamSubTxType_DualTx2, ///< Dual transmitter 1, e.g. 950A, 0x50 I2C device
} SamSubTxType;

/**********************************************************************
 * Globals
 **********************************************************************/

/**
 * We record the current mac state here so we know when we get a state change.
 */
MacCtxtState hSamSub_curState = MacCtxtState_Invalid;

/*
 * Trim/Level defaults
 */
#define TRIM_DEFAULT 0xB5

#define AGL_DEFAULT 0
#ifdef AGL_DEFAULT
#undef AGL_DEFAULT
#define AGL_DEFAULT 0xDB
#endif

/*
 * Must be indexed by SamSum_RegId enum.
 */
SamSub_CliReg samSub_cliRegs[] = {
    // All Speakers
    {SAMSUB_TWIREG_FAKE_STANDBY, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_ALL}, // must be listed first
    {SAMSUB_TWIREG_SPEAKER_MUTE, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_ALL},
    {SAMSUB_TWIREG_RX_FACTORY, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_ALL},
    //{SAMSUB_TWIREG_STREAM_DETECT, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_ALL},

    // Subwoofer
    {SAMSUB_TWIREG_RX_I2C_DISABLE, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_MSTR_VOL, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_MSTR_FINE_VOL, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_INPUT_MIX, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_EFFECT_MODE, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_TRIM_VOL, TRIM_DEFAULT, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_CUTOFF, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_VPS_OPTION, 1, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_LFP_START_STOP, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_LFP_REG0, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_LFP_REG1, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_LFP_REG2, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_LFP_REG3, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_LFP_REG4, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_LFP_REG5, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_LFP_REG6, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_LFP_REG7, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},

    // Rear Speakers
    {SAMSUB_TWIREG_REAR_MSTR_VOL, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_REAR_MSTR_FINE_VOL, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_REAR_INPUT_MIX, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_REAR_TOP_CUTOFF, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_LREAR_FRONT_TRIM, TRIM_DEFAULT, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_LREAR},
    {SAMSUB_TWIREG_RREAR_FRONT_TRIM, TRIM_DEFAULT, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_RREAR},
    {SAMSUB_TWIREG_LREAR_TOP_TRIM, TRIM_DEFAULT, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_LREAR},
    {SAMSUB_TWIREG_RREAR_TOP_TRIM, TRIM_DEFAULT, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_RREAR},
    {SAMSUB_TWIREG_LREAR_SIDE_TRIM, TRIM_DEFAULT, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_LREAR},
    {SAMSUB_TWIREG_RREAR_SIDE_TRIM, TRIM_DEFAULT, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_RREAR},
    {SAMSUB_TWIREG_REAR_MUTE, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},

    // DRC Registers
    {SAMSUB_TWIREG_SUBW_DRC_ENABLE, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_ATTACK1_TIME, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_RELEASE1_TIME, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_LIMIT1_THOLD, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_CMP1_THOLD, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_CMP1_HS_SLOPE, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_ATTACK2_TIME, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_RELEASE2_TIME, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_LIMIT2_THOLD, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_CMP2_THOLD, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},
    {SAMSUB_TWIREG_SUBW_CMP2_HS_SLOPE, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_SUB},

    {SAMSUB_TWIREG_REAR_DRC_ENABLE, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_REAR_ATTACK_TIME, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_REAR_RELEASE_TIME, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_REAR_LIMIT_THOLD, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_REAR_CMP_THOLD, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_REAR_CMP_HS_SLOPE, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_RTOP_LIMIT_THOLD, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_RTOP_CMP_THOLD, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_RTOP_CMP_HS_SLOPE, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_RSIDE_LIMIT_THOLD, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_RSIDE_CMP_THOLD, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},
    {SAMSUB_TWIREG_RSIDE_CMP_HS_SLOPE, 0, SAMSUB_REGSTATE_UNSET, SAMSUB_OTASI_REAR},

    {0,0,0}
};

#if defined(CONFIG_INCLUDE_GPIO_LOOPBACK) && (CONFIG_INCLUDE_GPIO_LOOPBACK != 0)
u8 hSamSub_gpioPairs[] = {HSAMSUB_MUTE_GPIO, HSAMSUB_IRQ_GPIO};
#endif

bool hSamSub_tooEarlyToPair = TRUE;
bool hSamSub_pairWanted;
bool hSamSub_pairStepPending = FALSE;   // TRUE
bool hSamSub_autoPairClientDropped = FALSE;
volatile u16 hSamSub_pairTimer, hSamSub_searchNpairTimer;
CmdQueueEntry hSamSub_pairStepCmd;
CmdQueueEntry hSamSub_timerpairStepCmd;
CmdQueueEntry pairSm_searchNpair_stepCommand;
CmdQueueEntry hSamSub_syncRegsCmd;
bool hSamSub_timerpairStepCmdPending;
//CmdQueueEntry hSamSub_linkStatusUpdateCmd;
//volatile u16 hSamSub_linkStatusUpdateTimer = 0;
u16 hSamSub_pairTimeout;
u8 hSamSub_pairFindNextClient = 0;
bool hSamSub_doAutoPairing = TRUE;
u8 hSamSub_doAutoPairing_appToken = 0;
bool hSamSub_autoPairingDone;
bool hSamSub_pairCmdPresent;
bool hSamSub_regUpdateScheduled;
bool hSamSub_StartPFScehduled;
u8 hSamSub_factoryMode = 0;
volatile u16 hSamSub_radioStabilizedTimer = 0;
CmdQueueEntry hSamSub_handleTwiWriteCmd;
bool hSamSub_radioStabilized;
bool hSamSub_radioEnabled;

/* The current setting of the fake standby command */
u8 hSamSub_FakeStbyState = 0;
u8 hSamSub_FakeStbyState_acks = 0;

/* A flag set to true when an update is in progress.  Used to stop other buffer/OTA use  temporarily to give better reliability. */
bool hUpdateImgTickle;

MacAudioTable fourchan_masterAudio = {
    {{12, 40, 40, 6, 0, 48000}, //  48000
     { 8, 16, 16, 4, 0, 32000}, // 32000
     { 6, 12, 12, 3, 0, 24000}, // 24000
     { 4,  8,  8, 2, 0, 16000}, // 16000
     { 4,  8,  8, 2, 0, 12000}, // 12000
    },
    2, // tx_aud_tr;
    1, // tx_voi_tr;
    1, // rx_aud_tr;
    2, // rx_voi_tr;
    4  // voi_blks
};

// Number of buffers to add
/// \todo This is likely excessive. We need to test to see how many we
/// actually need. We definitely need more than the 10 allocated by the
/// ROM, we were seeing asserts at that value.
#define CMD_BUFFER_POOL_NUM_EXT_BUFFERS 30    //Yamaha, CL248152 reduced to 10

// Buffer memory
CmdBuffer cmdBufferPoolExt[CMD_BUFFER_POOL_NUM_EXT_BUFFERS];

extern bool protectFlash; // flag to configure flash protection (or not)

#if (CONFIG_SUPPORT_RF_FUNCTIONS)
SamSub_RfAntenna hSamSub_RfTestAntenna = SAMSUB_RFTEST_ANTENNA_DIVERSITY; // default
u8 hSamSub_RfTestTxDigLevel;
#endif

bool doing_primary_auto_pairing = FALSE;
bool factory_fixid;  //Factory Fixed ID mode
bool pair_cli0_is_zero, pair_cli1_is_zero, pair_cli2_is_zero;
u16 clientVersion;
u8 cmdregs[5];
bool hSamSub_isAlwaysPair;

/**********************************************************************
 * Forward Declarations
 **********************************************************************/

void hSamSub_handleTwiWrite(void);

bool hSamSub_findClient(CmdNodeId node_id);

bool hSamSub_sendDataToClient(CmdNodeId node_id, u8 twiReg, u8 newVal);

void hSamSub_updateLinkStatusReg(void);

bool hSamSub_letRadioStabilize(void);

bool hSamSub_NumClientsConnected(void);

void hSamSub_handleStateChange(void);

void hSamSub_pair(void);

void hSamSub_syncRegs(void);

void hSamSub_enableRadio(void);

void cancelAndZeroTimer(u16 * tmr);

void hSamSub_setRearRightSpk_tx_power(void);

/**********************************************************************
 * ABSL Helper Functions
 **********************************************************************/

/*! \brief Count clean channels — those whose CQ metric MSB meets
 *  the ABSL_CLEAN_CQ_THLD threshold (same criterion as James' ABSL).
 */
static u8 abslCountCleanChans(void) {
    u8 ch, count = 0;
    for (ch = 0; ch < MAX_CHANNELS; ch++) {
        if (romCfgCtxt.userChannelAvoid[ch])
            continue;
        if ((u8)(macCqData.metrics[ch] >> 8) >= ABSL_CLEAN_CQ_THLD)
            count++;
    }
    return count;
}

/*! \brief Check if CQ metrics are ready after a full sniff pass.
 *  macCqSniffMerge() clears firstNbSample after processing one
 *  complete sample window across all channels.
 */
static bool abslMetricsReady(void) {
    return !macCqData.firstNbSample;
}

/*! \brief Switch to a new band for ABSL scanning.
 *  Clears CQ metrics, sets the new band, re-inits RF, and resets
 *  the search hop sequence to the new band.
 */
static void abslSwitchBand(MacHopBand sku) {
    u8 ch;
    /* Clear metrics so we get fresh data for the new band */
    for (ch = 0; ch < MAX_CHANNELS; ch++)
        macCqData.metrics[ch] = 0;
    macCqData.firstNbSample = TRUE;

    appCmn_selectBand(sku + 1);
    macHopSyncHop(MacHopSyncChReset);
}

/**********************************************************************
 * Functions
 **********************************************************************/
#if DEBUG_ASSERT_TWI
    void assertTwiNmiIsr(void) {
        ASSERT(FALSE);
    }
#endif

void appAreRxPairingDataZeros(void)
{
	pair_cli0_is_zero = (pairedCliId0.id.as_u32 == 0);
    pair_cli1_is_zero = (pairedCliId1.id.as_u32 == 0);
    pair_cli2_is_zero = (pairedCliId2.id.as_u32 == 0);
}

void appClearPairingData(void)
{
    myId.id.as_u32 = 0;
    pairedCliId0.id.as_u32 = 0;
    pairedCliId1.id.as_u32 = 0;
    pairedCliId2.id.as_u32 = 0;
    appCmnPairSuccess();
}

// Using these two functions saves 89 bytes...
bool appIsActiveLinks(void) {
    return ((hSamSub_curState == MacCtxtState_Active) ||
            (hSamSub_curState == MacCtxtState_ActiveSearch) ||
            (hSamSub_curState == MacCtxtState_ActivePair) ||
            (stwis_regs[SAMSUB_TWIREG_LINK_STATUS] != 0));
}

bool appIsPairMode(void) {
    return ((macCtxtData.state == MacCtxtState_Pair) ||
            (macCtxtData.state == MacCtxtState_PairViaSynchro) ||
            (macCtxtData.state == MacCtxtState_ActivePair));
}

void appUpdateTwiRegChecksum(void) {
    stwis_regs[SAMSUB_TWIREG_INIT_CHECKSUM] = stwis_regs[SAMSUB_TWIREG_CMD_FACTORY] +
            stwis_regs[SAMSUB_TWIREG_CMD_PAIR] + stwis_regs[SAMSUB_TWIREG_CMD_FREQ] +
            stwis_regs[SAMSUB_TWIREG_CMD_MODE];
}

void appCancelPairing(void) {
static u16 autopair_timeSpent = 0;
    // Reset some pairing variables, but not hSamSub_pairFindNextClient as we want to keep looking for unpaired clients

    if (doing_primary_auto_pairing) {
        hSamSub_pairCmdPresent = TRUE;
        hSamSub_pairTimeout = 2;
        autopair_timeSpent += 4;
        if (autopair_timeSpent >= 600)
        {
            doing_primary_auto_pairing = FALSE;
            hSamSub_pairCmdPresent = FALSE;
            hSamSub_pairTimeout = 0;
        }
    }
    else {
        hSamSub_pairTimeout = 0;
        hSamSub_pairCmdPresent = FALSE;
    }

    cancelAndZeroTimer((u16 *)&hSamSub_pairTimer);

    // For normal manual pairing initiated by the pair flag, clear the
    // pair flag when done
    if (!doing_primary_auto_pairing) {
        stwis_regs[SAMSUB_TWIREG_CMD_PAIR] = 0;
        appUpdateTwiRegChecksum();
    }

    if (appIsPairMode())
        macPairCancel();

    // In 250ms, force update of link status register... but why?
    //if (!hSamSub_linkStatusUpdateTimer) {
    //    hSamSub_linkStatusUpdateTimer = timerRequestmSec(FALSE, FALSE, (u16 *)&hSamSub_linkStatusUpdateCmd, 250);
    //    ASSERT(hSamSub_linkStatusUpdateTimer);
    //}
}

void hSamSub_endAutoPairCycle(void) {
    if (doing_primary_auto_pairing) {
        hSamSub_autoPairClientDropped = FALSE;
        doing_primary_auto_pairing = FALSE;
        appCancelPairing();
    }
}

void hSamSub_pairTimer_cb(void) {
    if (hSamSub_pairTimer != 0); {// If not cancelled, post the timer based step
        hSamSub_pairTimer = 0;
        if (!hSamSub_timerpairStepCmdPending) {
            hSamSub_timerpairStepCmdPending = TRUE;
            cmdSendCommand_L1(&hSamSub_timerpairStepCmd, CmdQueuePriority_LOW);
        }
    }
}

void hSamSub_macStandby(void) {
    hSamSub_endAutoPairCycle();
    appCmn_delayMs(20);
    macStandby();
}

/*
 * Main application command loop.
 */
void appMasterCommand(void)
{
    CmdQueueEntry *cmd, *resp;
    CmdBuffer *buf;
    u8 i;
    u16 noiseLevel;

#if DEBUG_ASSERT_TWI
    {
        static u8 i;

        // Override the NMI_Isr with our own ISR so that we can throw an
        // assert if an NMI_Isr tries to run. We do this as late during
        // startup as possible so that all modules have had a chance to
        // initialize the vector table.
        for (i = 0; i < Int_Num_Vectors; i += 1)
            if (pIntVectorBase->vector[i] == NMI_Isr)
                pIntVectorBase->vector[i] = (IntHardVector)assertTwiNmiIsr;
    }
#endif

#if DEBUG_STACK_MONITOR
    // Initializes the stack to a known value. Note that we aren't going
    // to return from this function, so we should be able to set the
    // stack back to STACK_BEGIN if need to reduce stack usage.
    STACK_INIT();
#endif

    do {
#if DEBUG_STACK_MONITOR
        // Checks the stack and asserts if it has reached a high
        // watermark.
        STACK_CHECK();
#endif

        cmd = cmdGetCommand();
        if (cmdIsExtendedCmd(cmd->buf.d.opcode) && (cmd->buf.ext.dst == CmdModuleId_TWIS) &&
            ((CmdRouteGetDestNodeId(cmd->buf.ext.node) == CmdNodeId_Source) ||
             (CmdRouteGetDestNodeId(cmd->buf.ext.node) == CmdNodeId_LOCAL))) {
            stwis_nqCmd(cmd);
        } else if ((cmd->buf.d.opcode == CmdCode_Extended_Query) &&
                   (CmdRouteGetModuleId(cmd->buf.ext.dst) == CmdModuleId_CmdLoop) &&
                   /* (((CmdLoopCommand *)cmd->pBuffer->buffer)->cmd == LoopCommand_GetVersion)) { */
                   (cmd->pBuffer->buffer[1] == LoopCommand_GetVersion)) {
            resp = cmdGet(CmdRouteIdProcMod(CONFIG_PROCESSOR_ID, CmdModuleId_CmdLoop));
            cmdBuildResponse(resp, cmd);
            buf = resp->pBuffer;
            buf->buffer[0] = 8; /* Length. */
            /* ->buffer[1] can be left as-is. */
            buf->buffer[2] = 0;
            buf->buffer[3] = 0;
            buf->buffer[4] = (SWA11VERSION & 0xff);
            buf->buffer[5] = (SWA11VERSION >> 8);
            buf->buffer[6] = PRODUCT_ID; // if PRODUCT_ID is undefined, it will default to zero.
            buf->buffer[7] = CmdNodeId_Source;
            cmdSendCommand_LT(resp, CmdQueuePriority_LOW);
            appCmdDone(cmd);
        } else if (cmdIsExtendedCmd(cmd->buf.d.opcode) &&
            (cmd->buf.ext.dst == CmdModuleId_App) &&
            ((CmdRouteGetDestNodeId(cmd->buf.ext.node) == CmdNodeId_Source) ||
             (CmdRouteGetDestNodeId(cmd->buf.ext.node) == CmdNodeId_LOCAL))) {
            /* Extended command for me. */
            buf = cmd->pBuffer;
            switch(buf->buffer[1]) {
            case SAMSUB_SUBCMDE_SET_REG_RESP:
                {
                    u8 cnodeId = CmdRouteGetSrcNodeId(cmd->buf.ext.node);

                    if (buf->buffer[2] == SAMSUB_TWIREG_FAKE_STANDBY) {
                        hSamSub_FakeStbyState_acks |= (1<< (cnodeId-1));
                    }
                }
                break;
#if CONFIG_SUPPORT_UPDATE
            case SAMSUB_SUBCMDE_UPDATE_START:
                hUpdateImgTickle = TRUE;
                updateImage_start();
                break;
            case SAMSUB_SUBCMDE_UPDATE_DATA:
                hUpdateImgTickle = TRUE;
                updateImage_data(&buf->buffer[2], buf->buffer[0] - 2);
                break;
            case SAMSUB_SUBCMDE_UPDATE_DONE:
                hUpdateImgTickle = TRUE;
                resp = cmdGet(CmdModuleId_CmdLoop);
                cmdBuildResponse(resp, cmd);
#if DEBUG_SEQUENCE_UPDATE
                buf->buffer[0] = 5;
#else
                buf->buffer[0] = 2;
#endif
                buf->buffer[1] = (updateImage_verify(TRUE
#if DEBUG_SEQUENCE_UPDATE
                                                     , buf->buffer
#endif
                                      )
                                  ? SAMSUB_SUBCMDE_UPDATE_SUCCESS
                                  : SAMSUB_SUBCMDE_UPDATE_FAIL);
                cmdSendCommand_LT(resp, CmdQueuePriority_LOW);
                break;
#endif // CONFIG_SUPPORT_UPDATE
            case SAMSUB_SUBCMDE_CLIENT_VERSION: {
                // static u8 match, nomatch;  // debug
                clientVersion = buf->buffer[3] + (buf->buffer[4] << 8);
                if (clientVersion == 0)
                    clientVersion = buf->buffer[2] + (buf->buffer[8] << 8);
                // See which client this message came from
                if (CmdNodeId_Listener_2 == buf->buffer[9]) { // rear left speaker
                    if (clientVersion < 1000) {
#ifdef LOADER_BLOCK_DEBUG
#warning -----------------------------LOADER_BLOCK_DEBUG ENABLED------------------------------
                        stwis_regs[SAMSUB_TWIREG_REAR_LEFT_VER1] = buf->buffer[8];
#else
                        stwis_regs[SAMSUB_TWIREG_REAR_LEFT_VER1] = appConvToAsciiCode(clientVersion, 100);
#endif
                        stwis_regs[SAMSUB_TWIREG_REAR_LEFT_VER2] = appConvToAsciiCode(clientVersion, 10);
                        stwis_regs[SAMSUB_TWIREG_REAR_LEFT_VER3] = appConvToAsciiCode(clientVersion, 1);
                        stwis_regs[SAMSUB_TWIREG_REAR_LEFT_VER4] = '0';
                    } else {
#ifdef LOADER_BLOCK_DEBUG
#warning -----------------------------LOADER_BLOCK_DEBUG ENABLED------------------------------
                        stwis_regs[SAMSUB_TWIREG_REAR_LEFT_VER1] = buf->buffer[8];
#else
                        stwis_regs[SAMSUB_TWIREG_REAR_LEFT_VER1] = appConvToAsciiCode(clientVersion, 1000);
#endif
                        stwis_regs[SAMSUB_TWIREG_REAR_LEFT_VER2] = appConvToAsciiCode(clientVersion, 100);
                        stwis_regs[SAMSUB_TWIREG_REAR_LEFT_VER3] = appConvToAsciiCode(clientVersion, 10);
                        stwis_regs[SAMSUB_TWIREG_REAR_LEFT_VER4] = appConvToAsciiCode(clientVersion, 1);
                    }
                    stwis_regs[SAMSUB_TWIREG_REAR_LEFT_VER5] = appConvToAsciiCode(buf->buffer[5], 1);  // test version release number

                    stwis_regs[SAMSUB_TWIREG_AMP_TYPE] |= ((stwis_regs[SAMSUB_TWIREG_AMP_TYPE] & ~0x0C) | ((buf->buffer[10] & 0x03) << 2));
                }
                else if (CmdNodeId_Listener_3 == buf->buffer[9]) { // rear right speaker
                    if (clientVersion < 1000) {
#ifdef LOADER_BLOCK_DEBUG
#warning -----------------------------LOADER_BLOCK_DEBUG ENABLED------------------------------
                        stwis_regs[SAMSUB_TWIREG_REAR_RIGHT_VER1] = buf->buffer[8];
#else
                        stwis_regs[SAMSUB_TWIREG_REAR_RIGHT_VER1] = appConvToAsciiCode(clientVersion, 100);
#endif
                        stwis_regs[SAMSUB_TWIREG_REAR_RIGHT_VER2] = appConvToAsciiCode(clientVersion, 10);
                        stwis_regs[SAMSUB_TWIREG_REAR_RIGHT_VER3] = appConvToAsciiCode(clientVersion, 1);
                        stwis_regs[SAMSUB_TWIREG_REAR_RIGHT_VER4] = '0';
                    } else {
#ifdef LOADER_BLOCK_DEBUG
#warning -----------------------------LOADER_BLOCK_DEBUG ENABLED------------------------------
                        stwis_regs[SAMSUB_TWIREG_REAR_RIGHT_VER1] = buf->buffer[8];
#else
                        stwis_regs[SAMSUB_TWIREG_REAR_RIGHT_VER1] = appConvToAsciiCode(clientVersion, 1000);
#endif
                        stwis_regs[SAMSUB_TWIREG_REAR_RIGHT_VER2] = appConvToAsciiCode(clientVersion, 100);
                        stwis_regs[SAMSUB_TWIREG_REAR_RIGHT_VER3] = appConvToAsciiCode(clientVersion, 10);
                        stwis_regs[SAMSUB_TWIREG_REAR_RIGHT_VER4] = appConvToAsciiCode(clientVersion, 1);
                    }
                    stwis_regs[SAMSUB_TWIREG_REAR_RIGHT_VER5] = appConvToAsciiCode(buf->buffer[5], 1); // test version release number

                    stwis_regs[SAMSUB_TWIREG_AMP_TYPE] |= ((stwis_regs[SAMSUB_TWIREG_AMP_TYPE] & ~0x30) | ((buf->buffer[10] & 0x03) << 4));

                    // For Rear Right, change the TX power in SKU-G
                    if (MacHopBand4C_SKU_G_IND == stwis_regs[SAMSUB_TWIREG_CMD_FREQ]) {
                        hSamSub_setRearRightSpk_tx_power();
                    }
                }
                else {    // subwoofer
                    if (clientVersion < 1000) {
                        stwis_regs[SAMSUB_TWIREG_SUBW_VER1] = appConvToAsciiCode(clientVersion, 100);
                        stwis_regs[SAMSUB_TWIREG_SUBW_VER2] = appConvToAsciiCode(clientVersion, 10);
                        stwis_regs[SAMSUB_TWIREG_SUBW_VER3] = appConvToAsciiCode(clientVersion, 1);
                        stwis_regs[SAMSUB_TWIREG_SUBW_VER4] = '0';
                    } else {
                        stwis_regs[SAMSUB_TWIREG_SUBW_VER1] = appConvToAsciiCode(clientVersion, 1000);
                        stwis_regs[SAMSUB_TWIREG_SUBW_VER2] = appConvToAsciiCode(clientVersion, 100);
                        stwis_regs[SAMSUB_TWIREG_SUBW_VER3] = appConvToAsciiCode(clientVersion, 10);
                        stwis_regs[SAMSUB_TWIREG_SUBW_VER4] = appConvToAsciiCode(clientVersion, 1);
                    }
                    stwis_regs[SAMSUB_TWIREG_SUBW_VER5] = appConvToAsciiCode(buf->buffer[5], 1);  // test version release number

                    stwis_regs[SAMSUB_TWIREG_AMP_TYPE] |= ((stwis_regs[SAMSUB_TWIREG_AMP_TYPE] & ~0x03) | (buf->buffer[10] & 0x03));
                }
            }   break;

            case SAMSUB_SUBCMDE_CLIENT_SUBEVENTS:

                // Cause all registers to be sent to the clients.
                if (buf->buffer[7] & BIT(0)) {
                    // hSamSub_syncRegs(); - Don't call direct and a) do unnecessary work, b) rather, make use of the indirect (static command) mutex.
                    hSamSub_handleStateChange();
                }

                // Subwoofer DSP Type
                stwis_regs[SAMSUB_TWIREG_SUBW_DSP_TYPE] = 0x01; //= NoDSP only

                // Rear or Sub Type
                if (CmdNodeId_Listener_2 == buf->buffer[2]) { // rear left speaker [3:0]
                    stwis_regs[SAMSUB_TWIREG_REAR_TYPE] &= ~(0x0F);
                    stwis_regs[SAMSUB_TWIREG_REAR_TYPE] |= (buf->buffer[10] & 0x0F); //= reg13_RearType
                    stwis_regs[SAMSUB_TWIREG_LREAR_RX_TYPE] = buf->buffer[11];
                    stwis_regs[SAMSUB_TWIREG_LR_ERRSTAT_FNT] = buf->buffer[12];
                    stwis_regs[SAMSUB_TWIREG_LR_ERRSTAT_TOP] = buf->buffer[13];
                    stwis_regs[SAMSUB_TWIREG_LR_ERRSTAT_SIDE] = buf->buffer[16];
                }
                else if ((CmdNodeId_Listener_3 == buf->buffer[2])) { // rear right speaker [7:4]
                    stwis_regs[SAMSUB_TWIREG_REAR_TYPE] &= ~(0xF0);
                    stwis_regs[SAMSUB_TWIREG_REAR_TYPE] |= (buf->buffer[10] & 0xF0); //= reg13_RearType
                    stwis_regs[SAMSUB_TWIREG_RREAR_RX_TYPE] = buf->buffer[11];
                    stwis_regs[SAMSUB_TWIREG_RR_ERRSTAT_FNT] = buf->buffer[12];
                    stwis_regs[SAMSUB_TWIREG_RR_ERRSTAT_TOP] = buf->buffer[13];
                    stwis_regs[SAMSUB_TWIREG_RR_ERRSTAT_SIDE] = buf->buffer[16];
                } else if ((CmdNodeId_Listener_1 == buf->buffer[2])) {
                    stwis_regs[SAMSUB_TWIREG_SUBW_RX_TYPE] = buf->buffer[11];
                    stwis_regs[SAMSUB_TWIREG_SUBW_ERRSTAT] = buf->buffer[17];
                }
                break;

            case SAMSUB_SUBCMDE_CLEAR_PAIR:
                hSamSub_macStandby();
                appClearPairingData();
                // Please do not add any RESET or the other process
                // Module manufacture uses this point as their module sanity check.
                break;

#if (defined CONFIG_APPCMD_OPT_OUT) && (CONFIG_APPCMD_OPT_OUT == 0)
            case SAMSUB_SUBCMDE_MFG_DEBUG:
                {
                    u8 addr = buf->buffer[3];
                    u8 len = buf->buffer[0] - 4; //len,op,sub,addr is 4

                    if ((SAMSUB_MFGCMD_TWIWR == buf->buffer[2]) && (addr+len < STWIS_NUM_REGS))
                    {
                        memCopy(&stwis_regs[addr], &buf->buffer[4], len);
                        //SAFEMEMCOPY(&stwis_regs[addr], &buf->buffer[4], len);
                        while (len-- && (addr+len <= STWIS_NUM_SETS))
                        {
                            stwis_set[addr+len] = 1;
                        }
                        hSamSub_handleTwiWrite();
                    }

                    else if (SAMSUB_MFGCMD_TWIRD == buf->buffer[2])
                    {
                        memCopy(&buf->buffer[4], &stwis_regs[addr], len);
                        //SAFEMEMCOPY(&buf->buffer[4], &stwis_regs[addr], len);
                    }

                }
                if (cmd->buf.d.opcode == CmdCode_Extended_Query)
                {
                    resp = cmdGet(CmdModuleId_CmdLoop);
                    cmdBuildResponse(resp, cmd);
                    cmdSendCommand_LT(resp, CmdQueuePriority_LOW);
                }
                break;
#endif  // (defined CONFIG_APPCMD_OPT_OUT) && (CONFIG_APPCMD_OPT_OUT == 0)

#if (defined CONFIG_SUPPORT_RF_FUNCTIONS) && (CONFIG_SUPPORT_RF_FUNCTIONS == 1)
            case SAMSUB_SUBCMDE_RF_TEST:
                appCmn_RfTest(buf);
                break;
#endif

#if DEBUG_ASSERT_TWI
            case SAMSUB_SUBCMDE_FORCE_ASSERT:
                ASSERT(0);
                break;
#endif
            }
            appCmdDone(cmd);
        } else if (cmd->buf.d.opcode == SAMSUB_CMDCODE_APP_SHORT) {
            switch(cmd->buf.d.buf[0]) {
            case SAMSUB_SUBCMDS_REG_CHANGE:
                stwis_handleRegChange();
                if (!hUpdateImgTickle)
                {
                    hSamSub_handleTwiWrite();
                }
                break;


            case SAMSUB_SUBCMDS_PAIR_STEP:
                if (!hSamSub_pairStepPending) {
                   break;
                }
                hSamSub_pairStepPending = FALSE;

            case SAMSUB_SUBCMDS_TIMERPAIR_STEP:
                if (SAMSUB_SUBCMDS_TIMERPAIR_STEP==cmd->buf.d.buf[0]) {
                    if (!hSamSub_timerpairStepCmdPending) {
                        break;
                    }
                    hSamSub_timerpairStepCmdPending = FALSE;;
                }

                /* ABSL: do not proceed to pairing while ABSL is still
                 * scanning.  Mirrors the synchronous gate in
                 * hSamSub_handleTwiWrite (~line 1370).  Without this,
                 * the orphan periodic timer set up in __init__ can
                 * fire its callback inside the ABSL window and
                 * prematurely transition MAC into Pair mode via
                 * hSamSub_pair() -> macPair(), leaving ABSL's PTM
                 * scan starved.  Re-armed automatically when ABSL
                 * reaches DONE; pairing proceeds normally then. */
                if (abslState == ABSL_SCAN_LO || abslState == ABSL_SCAN_HI) {
                    break;
                }

                if (hSamSub_tooEarlyToPair) {
                    if ((macCtxtData.state == MacCtxtState_Search) ||
                        (macCtxtData.state ==  MacCtxtState_SearchViaSynchro) ||
                        (macCtxtData.state == MacCtxtState_ActiveSearch)) {
                        hSamSub_tooEarlyToPair = FALSE;

                        /* Find the loudest channel, report it for RF testing. */
                        noiseLevel = 0x7fff;
                        for (i = 0; i < MAX_CHANNELS; ++i) {
                            u16 metric = macCqData.metrics[i];
                            if (metric < noiseLevel) {
                                noiseLevel = metric;
                            }
                        }
                        stwis_regs[SAMSUB_TWIREG_RF_NOISE_LEVEL_LO] = (u8)noiseLevel;
                        stwis_regs[SAMSUB_TWIREG_RF_NOISE_LEVEL_HI] = (u8)(noiseLevel >> 8);

                        if (hSamSub_pairWanted || SAMSUB_IS_ALWAYS_PAIR(myId.id.as_u32)) {
                            hSamSub_pair();
                        }
                    } else {
                        /* Try again later to see if we are ready to pair yet. */
                        hSamSub_pairTimer = timerRequestmSec(FALSE, TRUE, (u16 *)hSamSub_pairTimer_cb, 100);
                        ASSERT(hSamSub_pairTimer);
                    }
                }
                else if (SAMSUB_IS_ALWAYS_PAIR(myId.id.as_u32) && (3 != hSamSub_NumClientsConnected()))
                {
                    hSamSub_pair();
                }
                else if (appIsPairMode() || hSamSub_autoPairClientDropped) {
                    if (doing_primary_auto_pairing && hSamSub_autoPairClientDropped) {
                        // We will go back into pair after a couple seconds in search to pick up bouncing autopaired clients
                        if (0==hSamSub_searchNpairTimer) {
                            hSamSub_searchNpairTimer = timerRequestSecond(FALSE, FALSE, (u16 *)&pairSm_searchNpair_stepCommand, 2);
                            ASSERT(hSamSub_searchNpairTimer);

                            appCancelPairing();  // Cancel pairing to search
                        }
                    }

                    if (!hSamSub_autoPairClientDropped) { // Cancel pairing due to normal timout
                        appCancelPairing();
                    }
                     //If NOT ALWAYS_PAIR and no clients are currently in a link, then stop RF transmission
                    //if (!SAMSUB_IS_ALWAYS_PAIR(myId.id.as_u32) && (0 == hSamSub_NumClientsConnected())
                    //    && doing_primary_auto_pairing) {
                    //    hSamSub_macStandby();
                    //}
                }
                break;

            case SAMSUB_SUBCMDS_SEARCHNPAIR_STEP:
                hSamSub_searchNpairTimer = 0;
                if (!appIsPairMode() && doing_primary_auto_pairing)
                {
                    hSamSub_pair();
                }
                break;
            case SAMSUB_SUBCMDS_SYNC_REGS:
                hSamSub_regUpdateScheduled = FALSE;
                hSamSub_syncRegs();
                break;
            case SAMSUB_SUBCMDS_TWI_WRITE:
                hSamSub_radioStabilizedTimer = 0;
                hSamSub_radioStabilized = TRUE;
                hSamSub_handleTwiWrite();
                break;
//            case SAMSUB_SUBCMDS_UPDATE_LINK_STATUS:
//                hSamSub_linkStatusUpdateTimer = 0;
//                hSamSub_updateLinkStatusReg();
//                break;
            }

#if defined(CONFIG_INCLUDE_GPIO_LOOPBACK) && (CONFIG_INCLUDE_GPIO_LOOPBACK != 0)
        } else if (cmd->buf.d.opcode == SAMSUB_CMDCODE_LOOPBACK_TEST) {
            gpioLoopback_run(1, hSamSub_gpioPairs, TRUE, cmd);
            stwis_nqCmd(cmd);
#endif // CONFIG_INCLUDE_GPIO_LOOPBACK
        } else {
            if (cmd->buf.d.opcode == CmdCode_Link_State_Notify) {

                // We get an initial Linkstate notify with no clients connected as
                // an initial 'im in search' event for the app.
                if (macCtxtData.state != hSamSub_curState) {
                    hSamSub_curState = macCtxtData.state;
                }

                // Adjust token for autopairing retriers
                if ((cmd->buf.d.buf[0] == MacSubCmd_SlotStateChange) && (cmd->buf.d.buf[2] == MacSlotState_Active)) {

                    // Update token for autopair case resyncing
                    if (hSamSub_findClient(CmdNodeId_Listener_1)) {
                        romCfgCtxt.appToken &= ~SAMSUB_MACTOKEN_SUBWOOFER;
                    }
                    if (hSamSub_findClient(CmdNodeId_Listener_2)) {
                        romCfgCtxt.appToken &= ~SAMSUB_MACTOKEN_REARLEFT;
                    }
                    if (hSamSub_findClient(CmdNodeId_Listener_3)) {
                        romCfgCtxt.appToken &= ~SAMSUB_MACTOKEN_REARRIGHT;
                    }
                }

                // Clear out the fake standby client acks if the link changed.  The
                // Fake standby register is the only one the clients ack the receipt of
                // since its reception and timeliness is important.
                hSamSub_FakeStbyState_acks = 0;

                if (cmd->buf.d.buf[0] == MacSubCmd_PairUpdate) {
                    /* Pairing change. Un-write-protect the flash so it can be persisted. */

                    // TODO - For the case of autopair retry, we do not need to re-store an already stored client.
                    //  Check that the appCmnHandleCmd code that does the storing does this....
                    if (!appCmnHandleCmd(cmd)) {
                        cmdHandleCmdRom(cmd);
                        appCmdDone(cmd);
                    }

                    // Capture the state of pairedCliId0.id, pairedCliId1.id and pairedCliId2.id
                    // Must be done after appCmnHandleCmd
                    appAreRxPairingDataZeros();

                    // Cancel pair timeout timer
                    if (!hSamSub_autoPairClientDropped) {
                        cancelAndZeroTimer((u16 *)&hSamSub_pairTimer);
                    }

                    {
                        // Adjust the apptoken to prevent duplicate adds

                        // Find out which is the active peerCache
                        u8 peerCacheIdx = cmd->buf.d.buf[1];

                        // Sub is CmdNodeId_Listener_1, Rear left is _listener_2,  Rear right is listener_3
                        if (CmdNodeId_Listener_1 == peerCache[peerCacheIdx].netNodeId) {
                            romCfgCtxt.appToken &= ~SAMSUB_MACTOKEN_SUBWOOFER;  // prevent pairing additional subs

                        } else if (CmdNodeId_Listener_2 == peerCache[peerCacheIdx].netNodeId) {
                            romCfgCtxt.appToken &= ~SAMSUB_MACTOKEN_REARLEFT;

                        } else if (CmdNodeId_Listener_3 == peerCache[peerCacheIdx].netNodeId){
                            romCfgCtxt.appToken &= ~SAMSUB_MACTOKEN_REARRIGHT;
                        }
                    }


                    if (!hSamSub_autoPairClientDropped) {

                        // - this is normal autopairing where we are not in cycle recovery mode
                        // If we're supposed to find the next client automatically
                        // start doing so now...
                        if (hSamSub_pairFindNextClient)
                        {
                            hSamSub_pairFindNextClient--;
                            if (doing_primary_auto_pairing) {
                                hSamSub_pairTimeout = PRIM_AUTO_PAIRING_TIMEOUT_SECS;
                            } else {
                                hSamSub_pairTimeout = 5;
                            }
                            hSamSub_pair();
                        } else {
                            hSamSub_doAutoPairing = FALSE;
                            appCancelPairing();
                        }
                    }

                    else {
                        // if hSamSub_autoPairClientDropped we are running the
                        // the cycle recovery so don't cancel or pair here because it
                        // will in the pair step routine.
                    }

                }

                hSamSub_updateLinkStatusReg();

                hSamSub_handleStateChange();

                // Handle factory ALWAYS_PAIR mode
                // If ALWAYS_PAIR and no clients are currently in a link with host, then pair -- ??? This comment does not match previous code && && &&
                if (SAMSUB_IS_ALWAYS_PAIR(myId.id.as_u32) && (3 != hSamSub_NumClientsConnected()))
                {
                    if (!hSamSub_pairStepPending)
                    {
                        hSamSub_pairStepPending = TRUE;
                        cmdSendCommand_LT(&hSamSub_pairStepCmd, CmdQueuePriority_LOW);
                    }
                }
            }
            else if (cmd->buf.d.opcode == CmdCode_Pair)
            {
                // Special case to help support DLL pair command. Pass in MODE as argument
                stwis_regs[SAMSUB_TWIREG_CMD_MODE] = cmd->buf.d.buf[0];
                stwis_regs[SAMSUB_TWIREG_CMD_PAIR] = 1;
                appUpdateTwiRegChecksum();
                if (!hUpdateImgTickle)
                {
                    hSamSub_handleTwiWrite();
                }
            }
            else
            {
                if (cmdIsExtendedCmd(cmd->buf.d.opcode) &&
                (cmd->buf.ext.dst == CmdModuleId_App) &&
                ((buf->buffer[1] >= SAMSUB_SUBCMDE_UPDATE_START) &&
                 (buf->buffer[1] <= SAMSUB_SUBCMDE_UPDATE_FAIL)))
                {
                    hUpdateImgTickle = TRUE;
                }
            }
            if (!appCmnHandleCmd(cmd)) {
                cmdHandleCmdRom(cmd);
                appCmdDone(cmd);
            }
        }

        /* ABSL: check if band scan metrics are ready each iteration */
        if (abslState == ABSL_SCAN_LO && abslMetricsReady()) {
            abslLoCleanChans = abslCountCleanChans();
            abslSwitchBand(abslHiSku);
            abslState = ABSL_SCAN_HI;
        } else if (abslState == ABSL_SCAN_HI && abslMetricsReady()) {
            u8 hiClean = abslCountCleanChans();
            if (abslLoCleanChans >= hiClean) {
                abslSwitchBand(abslLoSku);
            }
            abslState = ABSL_DONE;
            stwis_regs[SAMSUB_TWIREG_ABSL_STATUS] = ABSL_STATUS_DONE;
            hSamSub_handleTwiWrite();
        }
    } while(1);
}

u8 hSamSub_IsValidClientType(u8 mode) {
    u8 type;

    switch (mode) {
    case SAMSUB_CLIENT_TYPE0:
    case SAMSUB_CLIENT_TYPE1:
    case SAMSUB_CLIENT_TYPE2:
    case SAMSUB_CLIENT_TYPE3:
    case SAMSUB_CLIENT_TYPE4:
    case SAMSUB_CLIENT_TYPE5:
        type = TRUE;
        break;
    default:
        type = FALSE;
    }
    return type;
}

void hSamSub_SetAppToken(CmdNodeId nodeId, u8 tokenBit, bool unpaired) {

    // Save current app token
    if (hSamSub_isAlwaysPair || factory_fixid) {
        romCfgCtxt.appToken |= tokenBit;
    } else {
        if (!hSamSub_pairCmdPresent) {
            if (cmdregs[SAMSUB_TWIREG_CMD_PAIR]) {
                if (!hSamSub_findClient(nodeId)) {
                        romCfgCtxt.appToken |= tokenBit;
                    }
            } else {
                if (unpaired) {
                    romCfgCtxt.appToken |= tokenBit;
                }
            }
        }
    }
}

void hSamsub_fcty_setfixedId(void) {
    myId.id.as_u32 = SAMSUB_FACTORY_HOST_ID | cmdregs[SAMSUB_TWIREG_CMD_FREQ];
    pairedCliId0.id.as_u32 = SAMSUB_FACTORY_CLIENT_ID_SUB | cmdregs[SAMSUB_TWIREG_CMD_FREQ];
    pairedCliId1.id.as_u32 = SAMSUB_FACTORY_CLIENT_ID_REAR | cmdregs[SAMSUB_TWIREG_CMD_FREQ];
    pairedCliId2.id.as_u32 = SAMSUB_FACTORY_CLIENT_ID_REAR_R | cmdregs[SAMSUB_TWIREG_CMD_FREQ];
}

void hSamsub_fcty_setfixedFreq(u8 chan) {
    g_lockChanMC.f[0] = chan & SAMSUB_FREQ_MASK;
    g_lockChanMC.f[1] = HOP_CHAN_INVALID;
    g_lockChanMC.f[2] = HOP_CHAN_INVALID;
    g_lockChannelMCMode = 1; // Sing chan
    g_forceHop = TRUE; // If the link is up we may have to hop there
}

void hSamSub_handleTwiWrite(void) {
    u8 twiReg, newVal;
    SamSub_CliReg *cr;
    u8 otasiSendsPerThread;
    u8 maxClients;
    SamSubTxType txType;

#if defined(MFCFG_MAC_FHSS) && (MFCFG_MAC_FHSS == 1)
    u8 fhssCh;
#endif

    cmdregs[0] = stwis_regs[0];
    cmdregs[1] = stwis_regs[1];
    cmdregs[2] = stwis_regs[2];
    cmdregs[3] = stwis_regs[3];
    cmdregs[4] = stwis_regs[4];

    /* We know that a TWI register changed. But we don't know which one! */
    if (hSamSub_IsValidClientType(cmdregs[SAMSUB_TWIREG_CMD_MODE])
        && (cmdregs[0] + cmdregs[1] + cmdregs[2] + cmdregs[3] == cmdregs[SAMSUB_TWIREG_INIT_CHECKSUM])) {

        appAreRxPairingDataZeros();

        /* We have a valid 5-byte command. */
        if (hSamSub_factoryMode != cmdregs[SAMSUB_TWIREG_CMD_FACTORY]) {
            hSamSub_factoryMode = cmdregs[SAMSUB_TWIREG_CMD_FACTORY];
            switch(hSamSub_factoryMode) {
            case SAMSUB_FACTORY_NONE:
                g_lockChannelMCMode = 0;
                g_forceHop = FALSE;
                // Break link if necessary to clear fixed chan & resume normal hopping
                macSearch();
                break;
#if (defined CONFIG_APPCMD_OPT_OUT) && (CONFIG_APPCMD_OPT_OUT == 0)
            case SAMSUB_FACTORY_CELL_TEST_FIX_FREQ_ID:
                hSamsub_fcty_setfixedId();
                /* Fall through to SAMSUB_FACTORY_AGING_TEST_FIX_FREQ case. */
            case SAMSUB_FACTORY_AGING_TEST_FIX_FREQ:
                // Set desired channel(s) in locked set
                hSamsub_fcty_setfixedFreq(cmdregs[SAMSUB_TWIREG_CMD_FREQ]);
                macSearch();
                break;
            case SAMSUB_FACTORY_REFERENCE_FOR_RX_TEST:
                myId.id.as_u32 = SAMSUB_FACTORY_ALWAYS_PAIR_ID | cmdregs[SAMSUB_TWIREG_CMD_FREQ];
                appCmnPairSuccess();
                hSamSub_isAlwaysPair = TRUE;
                hSamSub_autoPairingDone = TRUE;
                hSamSub_pairWanted = TRUE;
                if (!hSamSub_pairStepPending)
                {
                    hSamSub_pairStepPending = TRUE;
                    cmdSendCommand_LT(&hSamSub_pairStepCmd, CmdQueuePriority_LOW);
                }
                break;
            case SAMSUB_FACTORY_CELL_TEST_FIXID:
                /* Same as SAMSUB_FACTORY_CELL_TEST_FIX_FREQ_ID mode, but without the fixed channel. */
                hSamsub_fcty_setfixedId();
                factory_fixid = TRUE;
                macSearch();
                break;
#endif   // (defined CONFIG_APPCMD_OPT_OUT) && (CONFIG_APPCMD_OPT_OUT == 0)
            case SAMSUB_FACTORY_RESET: /* End of factory testing! Shut down, get ready for a fresh start. */
                hSamSub_macStandby();
                stwis_regs[SAMSUB_TWIREG_CMD_FACTORY] = 0;
                stwis_regs[SAMSUB_TWIREG_LINK_STATUS] = 0;
                hSamSub_factoryMode = 0; /* Make sure we can re-do a reset. Just in case. */
                appClearPairingData();
                ASSERT(FALSE);
                RegWrite(CHIP, CHIPRESET_REG, RESETSYS_S(1));
                //the chip reset command has been failed as Soundbar keeps the module chip reset pin(RESETN) high.
                //In this case, stop taking any further actions and go back to the task queue
                return;
            }
        }

        // Set appropriate MAC Token based on command mode
        // NOTE: These are the appTokens used by auto-pairing
        // Also NOTE: The appToken needs to be set properly BEFORE pairing
        // occurs. And the appToken is ONLY used at pairing time. Once clients
        // have been joined via pairing, appToken is ignored.
        //
        // Allow auto-pairing appToken mask bits based on what we've already
        // learned to date, but for manual pairing allow appToken mask based
        // on what we are currently linked with. This allows for replacement
        // speakers to be purchased later and joined into a network that was
        // previously "full".
        if (!hSamSub_pairCmdPresent) {
            // See if we should auto-pair
            hSamSub_doAutoPairing = pair_cli0_is_zero && pair_cli1_is_zero && pair_cli2_is_zero;
            romCfgCtxt.appToken = 0;
        }

        // Set the default value for our max number of clients. It will
        // get updated in the switch below if necessary.
        maxClients = 1;

        // Set the default value for our transmitter type.
        txType = SamSubTxType_SingleTx;

        switch (cmdregs[SAMSUB_TWIREG_CMD_MODE]) {
        case SAMSUB_MODE_REARRIGHT_ONLY:
            txType = SamSubTxType_DualTx2;
            hSamSub_SetAppToken(CmdNodeId_Listener_3, SAMSUB_MACTOKEN_REARRIGHT, pair_cli2_is_zero);
            break;

        case SAMSUB_MODE_SUB_PLUS_REARLEFT:
            txType = SamSubTxType_DualTx1;
            maxClients = 2;
            hSamSub_SetAppToken(CmdNodeId_Listener_1, SAMSUB_MACTOKEN_SUBWOOFER, pair_cli0_is_zero);
            hSamSub_SetAppToken(CmdNodeId_Listener_2, SAMSUB_MACTOKEN_REARLEFT,  pair_cli1_is_zero);
            break;

        case SAMSUB_MODE_SUB_REAR_READY:
            maxClients = 3;
            hSamSub_SetAppToken(CmdNodeId_Listener_1, SAMSUB_MACTOKEN_SUBWOOFER, pair_cli0_is_zero);

            // if primary auto-pairing, we need to look for rear speakers too
            hSamSub_SetAppToken(CmdNodeId_Listener_2, SAMSUB_MACTOKEN_REARLEFT, hSamSub_doAutoPairing && pair_cli1_is_zero);
            hSamSub_SetAppToken(CmdNodeId_Listener_3, SAMSUB_MACTOKEN_REARRIGHT, hSamSub_doAutoPairing && pair_cli2_is_zero);
            break;

        case SAMSUB_MODE_SUB_PLUS_REARLEFT_AND_REARRIGHT:
        case SAMSUB_MODE_SUB_PLUS_RL_AND_RR_READY:
        case SAMSUB_MODE_READY:
            maxClients = 3;
            hSamSub_SetAppToken(CmdNodeId_Listener_1, SAMSUB_MACTOKEN_SUBWOOFER, pair_cli0_is_zero);
            hSamSub_SetAppToken(CmdNodeId_Listener_2, SAMSUB_MACTOKEN_REARLEFT,  pair_cli1_is_zero);
            hSamSub_SetAppToken(CmdNodeId_Listener_3, SAMSUB_MACTOKEN_REARRIGHT, pair_cli2_is_zero);
            break;

        default:
            break;
        }

        // Only let the code below run once.
        // TODO: This code really needs to get re worked. We shouldn't
        // be running the same code over and over every time there is an
        // I2C read or write, when the user isn't trying to start the
        // radio.
        if (!hSamSub_radioEnabled) {
            if (hSamSub_isAlwaysPair || factory_fixid) {
                maxClients++;
            }
            macSetMaxLinks(maxClients);

            // Set up operating band before we start searching
            if (SAMSUB_FACTORY_NONE == cmdregs[SAMSUB_TWIREG_CMD_FACTORY]) {
                MacHopBand band = cmdregs[SAMSUB_TWIREG_CMD_FREQ];

                // To be compatible with release R684, we must stay with existing SKUs
                if (MacHopBand4C_SKU_F_IND == band) {
                    band = MacHopBand4C_SKU_E_IND;
                } else if (MacHopBand4C_SKU_G_IND == band) {
                    band = MacHopBand4A_SKU_C_IND;
                }
                appCmn_selectBand(band);

                /* ABSL: if reg 0x28 has a valid Lo/Hi SKU pair, override
                 * the boot band with Lo SKU to begin dual-band scan.
                 * ABSL is not used in FHSS or DSBEU builds (fixed band). */
#if !(defined(MFCFG_MAC_FHSS) && (MFCFG_MAC_FHSS == 1)) && !(defined(MFCFG_MAC_DSBEU) && (MFCFG_MAC_DSBEU == 1))
                {
                    u8 abslReg = stwis_regs[SAMSUB_TWIREG_STREAM_DETECT];
                    if (abslReg != 0x00 && abslReg != 0xFF) {
                        switch (abslReg & 0x0F) {
                            case 1: abslLoSku = MacHopBand1_SKU_D;  break;
                            case 2: abslLoSku = MacHopBand4C_SKU_H; break;
                            case 3: abslLoSku = MacHopBand4C_SKU_I; break;
                            case 4: abslLoSku = MacHopBand4C_SKU_J; break;
                            default: break;
                        }
                        switch ((abslReg >> 4) & 0x0F) {
                            case 1: abslHiSku = MacHopBand4A_SKU_C; break;
                            case 2: abslHiSku = MacHopBand4B_SKU_B; break;
                            case 3: abslHiSku = MacHopBand4C_SKU_A; break;
                            case 4: abslHiSku = MacHopBand4C_SKU_E; break;
                            case 5: abslHiSku = MacHopBand4C_SKU_F; break;
                            case 6: abslHiSku = MacHopBand4C_SKU_G; break;
                            default: break;
                        }
                        if (abslLoSku != MacHopBandInvalid && abslHiSku != MacHopBandInvalid) {
                            /* Use the same helper as the Lo->Hi mid-scan
                             * transition.  This clears stale CQ metrics
                             * and resets firstNbSample so the Lo scan
                             * actually waits for a real sample window
                             * instead of fast-completing with zeros. */
                            abslSwitchBand(abslLoSku);
                            abslState = ABSL_SCAN_LO;
                            stwis_regs[SAMSUB_TWIREG_ABSL_STATUS] = ABSL_STATUS_IN_PROGRESS;
                        }
                    }
                }
#endif
            }

#if defined(MFCFG_MAC_DSBEU) && (MFCFG_MAC_DSBEU == 1)
            // Set our channel map differently depending on our
            // transmitter type for the DSBEU/CCA build. Yes this is a
            // hack, we should solve the underlying issue, but
            // nevertheless we saw better performance like this.
            if (txType == SamSubTxType_SingleTx) {
                // Enable all channels except the last (49). Not sure
                // why so many more channels are available for single TX
                // versus dual TX.
                memSet(&romCfgCtxt.userChannelAvoid[0], MAX_CHANNELS-1, FALSE);
            } else if (txType == SamSubTxType_DualTx1) {
                romCfgCtxt.userChannelAvoid[0] = FALSE;
                romCfgCtxt.userChannelAvoid[1] = FALSE;
                romCfgCtxt.userChannelAvoid[2] = FALSE;
                romCfgCtxt.userChannelAvoid[3] = FALSE;
                romCfgCtxt.userChannelAvoid[19] = FALSE;
                romCfgCtxt.userChannelAvoid[39] = FALSE;
            } else { // (txType == SamSubTxType_DualTx2)
                romCfgCtxt.userChannelAvoid[6] = FALSE;
                romCfgCtxt.userChannelAvoid[7] = FALSE;
                romCfgCtxt.userChannelAvoid[8] = FALSE;
                romCfgCtxt.userChannelAvoid[9] = FALSE;
                romCfgCtxt.userChannelAvoid[29] = FALSE;
                romCfgCtxt.userChannelAvoid[48] = FALSE;
            }
#elif defined(MFCFG_MAC_FHSS) && (MFCFG_MAC_FHSS == 1)
            if (txType == SamSubTxType_SingleTx)
            {
                // Initially avoid Wi-Fi channels at 5815 MHz and below.
                fhssCh = MAX_CHANNELS - FHSS_MIN_NUM_CHANNELS;
            }
            else if (txType == SamSubTxType_DualTx1)
            {
                // Keep TX1 and TX2 on separate channels so that they
                // don't interfere with each other.
                memSet(&romCfgCtxt.userChannelAvoid[0], 13 - 0, TRUE);
                memSet(&romCfgCtxt.userChannelAvoid[31], 40 - 31, TRUE);

                // Initially avoid Wi-Fi channels at 5815 MHz and below.
                // Yes, this channel number is blocked, the code below
                // accommodates this.
                fhssCh = MAX_CHANNELS - FHSS_MIN_NUM_CHANNELS;
            }
            else // (txType == SamSubTxType_DualTx2)
            {
                // Set the band. We are setting the band without calling
                // appCmn_selectBand and macInitBand here, which might
                // leave some band settings unset. This could possibly
                // be a good thing in this case though; perhaps the FHSS
                // default (SKU B) power is preferred (the power is
                // currently the same for both SKUs, so this point is
                // moot, but that may change in the future).
                romCfgCtxt.activeBandIdx = MacHopBand4A_SKU_C;

                // Keep TX1 and TX2 on separate channels so that they
                // don't interfere with each other.
                memSet(&romCfgCtxt.userChannelAvoid[23], 43 - 23, TRUE);
                // RM #4371
                romCfgCtxt.userChannelAvoid[0] = TRUE;

                // Avoid the common Wi-Fi frequencies ranging from 5733
                // MHz to to 5815 MHz. Setting to 43 will catch the
                // channels at the end our range, as well as a few at
                // the beginning.
                fhssCh = 46;
            }

            // Select our initial channels.
            {
                u8 remaining = FHSS_MIN_NUM_CHANNELS;

                while (remaining)
                {
                    if (!romCfgCtxt.userChannelAvoid[fhssCh])
                    {
                        // Initialize our list of best channels. We
                        // don't really know which channels are clean
                        // yet, we'll learn that after we start
                        // sniffing. It would definitely be better if we
                        // made this decision after the initial scan.
                        // Here we selectively choose channels that our
                        // outside of common Wi-Fi space.
                        macCqLTData.channelFlags[fhssCh] |= MacCqLTChannelFlags_FHSSBesty;

                        remaining -= 1;
                    }

                    fhssCh += 1;

                    if (fhssCh >= MAX_CHANNELS)
                    {
                        fhssCh = 0;
                    }
                }
            }
#else
            if (romCfgCtxt.activeBandIdx == MacHopBand1_SKU_D) {
                // Canada IC (RM3707)
                romCfgCtxt.userChannelAvoid[0] = TRUE;
            } else if (cmdregs[SAMSUB_TWIREG_CMD_FREQ] == MacHopBand4C_SKU_F_IND) {
                memSet(&romCfgCtxt.userChannelAvoid[19], 10, TRUE);
                romCfgCtxt.userChannelAvoid[0] = TRUE;
            } else if (cmdregs[SAMSUB_TWIREG_CMD_FREQ] == MacHopBand4C_SKU_G_IND) {
                RegWrite(PHY_CONFIG, TX_DIG_LEVEL_REG, 27);
                memSet(&romCfgCtxt.userChannelAvoid[22], 22, TRUE);
                romCfgCtxt.userChannelAvoid[0] = TRUE;    //RM 4779
            }
#endif
        }

        hSamSub_enableRadio();

        /* ABSL: do not proceed to pairing until ABSL completes */
        if (abslState == ABSL_SCAN_LO || abslState == ABSL_SCAN_HI) {
            return;
        }

        if (!hSamSub_letRadioStabilize()) {
            // The radio isn't stabilized yet. We'll let the next call
            // to this function handle the code below.
            return;
        }

        // Now handle auto-pairing vs. manual pairing based on the mode we're in
        // and the clients we have learned or not...
        // NOTE: auto-pairing is based on what has previously been learned,
        // where manual pairing is based on what we are currently in a link with.
        if (!hSamSub_pairCmdPresent && !hSamSub_isAlwaysPair) {
            switch (cmdregs[SAMSUB_TWIREG_CMD_MODE]) {
            // This mode is used for Dual Tx (Right), so this rear speaker is
            // the rear right speaker.
            case SAMSUB_MODE_REARRIGHT_ONLY:
                // Handle primary auto-pairing
                if (!hSamSub_autoPairingDone && hSamSub_doAutoPairing) {
                    hSamSub_autoPairingDone = TRUE;
                    hSamSub_pairCmdPresent = TRUE;  // Block repeated pairing from hSamSub_syncRegs()
                    // Look for one client below, plus zero more
                    hSamSub_pairFindNextClient = 0;
                    hSamSub_pairTimeout = PRIM_AUTO_PAIRING_TIMEOUT_SECS;
                    hSamSub_pair();
                    doing_primary_auto_pairing = TRUE;
                }
                // Handle manual pairing
                else if (cmdregs[SAMSUB_TWIREG_CMD_PAIR]) {
                    u8 links = hSamSub_NumClientsConnected();
                    // If we have all expected clients, just exit
                    if (1 == links) {
                        hSamSub_pairWanted = FALSE;
                        break;
                    }
                    hSamSub_pairCmdPresent = TRUE;  // Block repeated pairing from hSamSub_syncRegs()
                    // If we have no linked clients, look for a total of
                    // 1 client.
                    if (!links) {
                        // Look for one client below, plus zero more
                        hSamSub_pairFindNextClient = 0;
                    }
                    hSamSub_pairTimeout = MANUAL_PAIRING_TIMEOUT_SECS;
                    hSamSub_pair();
                }
                // Handle secondary auto-pairing - nothing to do for this mode
                // else {}
                break;

            // This mode is used for Dual Tx (Left), so this rear speaker is
            // the rear left speaker and the sub-woofer.
            case SAMSUB_MODE_SUB_PLUS_REARLEFT:
                // Handle primary auto-pairing
                if (!hSamSub_autoPairingDone && hSamSub_doAutoPairing) {
                    hSamSub_autoPairingDone = TRUE;
                    hSamSub_pairCmdPresent = TRUE;  // Block repeated pairing from hSamSub_syncRegs()
                    // Look for one client below, plus one more
                    hSamSub_pairFindNextClient = 1;
                    hSamSub_pairTimeout = PRIM_AUTO_PAIRING_TIMEOUT_SECS;
                    hSamSub_pair();
                    doing_primary_auto_pairing = TRUE;
                }
                // Handle manual pairing
                else if (cmdregs[SAMSUB_TWIREG_CMD_PAIR]) {
                    u8 links = hSamSub_NumClientsConnected();
                    // If we have all expected clients, just exit
                    if (2 == links) {
                        hSamSub_pairWanted = FALSE;
                        break;
                    }
                    hSamSub_pairCmdPresent = TRUE;  // Block repeated pairing from hSamSub_syncRegs()
                    // If we have no linked clients, look for a total of
                    // 2 clients, otherwise just look for 1 client.
                    if (!links) {
                        // Look for one client below, plus one more
                        hSamSub_pairFindNextClient = 1;
                    }
                    else {
                        // Look for one client below, plus zero more
                        hSamSub_pairFindNextClient = 0;
                    }
                    hSamSub_pairTimeout = MANUAL_PAIRING_TIMEOUT_SECS;
                    hSamSub_pair();
                }
                // Handle secondary auto-pairing
                else {
                    if (!hSamSub_autoPairingDone) {
                        hSamSub_autoPairingDone = TRUE;
                        // If we haven't learned all expected clients, we auto-pair...
                        if (pair_cli0_is_zero || pair_cli1_is_zero) {
                            // Look for one client below, plus zero more
                            hSamSub_pairFindNextClient = 0;
                            hSamSub_pairTimeout = SECOND_AUTO_PAIRING_TIMEOUT_SECS;
                            hSamSub_pair();
                        }
                    }
                }
                break;

            // This mode is used for Single Tx. This auto-pairs the sub-
            // woofer, but will allow manual pairing of the rear speakers.
            case SAMSUB_MODE_SUB_REAR_READY:
                // Handle primary auto-pairing
                if (!hSamSub_autoPairingDone && hSamSub_doAutoPairing) {
                    hSamSub_autoPairingDone = TRUE;
                    hSamSub_pairCmdPresent = TRUE;  // Block repeated pairing from hSamSub_syncRegs()
                    // Look for one client below, plus two more
                    hSamSub_pairFindNextClient = 2;
                    hSamSub_pairTimeout = PRIM_AUTO_PAIRING_TIMEOUT_SECS;
                    hSamSub_pair();
                    doing_primary_auto_pairing = TRUE;
                }
                // Handle manual pairing
                else if (cmdregs[SAMSUB_TWIREG_CMD_PAIR]) {
                    u8 links = hSamSub_NumClientsConnected();
                    // If we have all expected clients, just exit
                    if (3 == links) {
                        hSamSub_pairWanted = FALSE;
                        break;
                    }
                    hSamSub_pairCmdPresent = TRUE;  // Block repeated pairing from hSamSub_syncRegs()
                    // In manual pair mode, if at least one client is actively connected,
                    //  look for the other two. Otherwise look for all three...
                    // No clients are connected yet
                    if (!links) {
                        // Look for one client below, plus two more
                        hSamSub_pairFindNextClient = 2;
                    }
                    else {
                        // Look for one client below, plus zero more
                        hSamSub_pairFindNextClient = 0;
                        if (1 == links) {
                            // We have one client actively connected.
                            // Look for one more client below, plus one more
                            hSamSub_pairFindNextClient = 1;
                        }
                    }
                    hSamSub_pairTimeout = MANUAL_PAIRING_TIMEOUT_SECS;
                    hSamSub_pair();
                }
                // Handle secondary auto-pairing
                else {
                    if (!hSamSub_autoPairingDone) {
                        hSamSub_autoPairingDone = TRUE;
                        // If we haven't learned all expected clients, we auto-pair...
                        if (pair_cli0_is_zero) {
                            // Look for one client below, plus zero more
                            hSamSub_pairFindNextClient = 0;
                            hSamSub_pairTimeout = SECOND_AUTO_PAIRING_TIMEOUT_SECS;
                            hSamSub_pair();
                        }
                    }
                }
                break;

            // This mode is used for Single Tx. This auto-pairs the sub-
            // woofer and the rear speakers.
            case SAMSUB_MODE_SUB_PLUS_REARLEFT_AND_REARRIGHT:
                // Handle primary auto-pairing
                if (!hSamSub_autoPairingDone && hSamSub_doAutoPairing) {
                    hSamSub_autoPairingDone = TRUE;
                    hSamSub_pairCmdPresent = TRUE;  // Block repeated pairing from hSamSub_syncRegs()
                    // Look for one client below, plus two more
                    hSamSub_pairFindNextClient = 2;
                    hSamSub_pairTimeout = PRIM_AUTO_PAIRING_TIMEOUT_SECS;
                    hSamSub_pair();
                    doing_primary_auto_pairing = TRUE;
                }
                // Handle manual pairing
                else if (cmdregs[SAMSUB_TWIREG_CMD_PAIR]) {
                    u8 links = hSamSub_NumClientsConnected();
                    // If we have all expected clients, just exit
                    if (3 == links) {
                        hSamSub_pairWanted = FALSE;
                        break;
                    }
                    hSamSub_pairCmdPresent = TRUE;  // Block repeated pairing from hSamSub_syncRegs()
                    // We have none of the clients linked
                    if (!links) {
                        // Look for one client below, plus two more
                        hSamSub_pairFindNextClient = 2;
                    }
                    // We have some of the clients linked
                    else {
                        // Look for one client below, plus zero more
                        hSamSub_pairFindNextClient = 0;
                        if (1 == links) {
                            // Look for one client below, plus one more
                            hSamSub_pairFindNextClient = 1;
                        }
                    }
                    hSamSub_pairTimeout = MANUAL_PAIRING_TIMEOUT_SECS;
                    hSamSub_pair();
                }
                // Handle secondary auto-pairing
                else {
                    if (!hSamSub_autoPairingDone) {
                        hSamSub_autoPairingDone = TRUE;
                        // If we haven't learned all expected clients, we auto-pair...
                        if (pair_cli0_is_zero || pair_cli1_is_zero || pair_cli2_is_zero) {
                            u8 missing_clients = 0;
                            if (pair_cli0_is_zero) {
                                missing_clients++;
                            }
                            if (pair_cli1_is_zero) {
                                missing_clients++;
                            }
                            if (pair_cli2_is_zero) {
                                missing_clients++;
                            }
                            hSamSub_pairFindNextClient = missing_clients - 1;
                            hSamSub_pairTimeout = SECOND_AUTO_PAIRING_TIMEOUT_SECS;
                            hSamSub_pair();
                        }
                    }
                }
                break;

            // This mode is used for 3 clients, sub + RL + RR. It supports two pairing modes:
            // secondary auto-pairing and manual pairing
            case SAMSUB_MODE_SUB_PLUS_RL_AND_RR_READY:
            // This mode only manually pairs in one client.
            case SAMSUB_MODE_READY:
                // Handle primary auto-pairing - nothing to do for this mode
                // Handle manual pairing
                if (cmdregs[SAMSUB_TWIREG_CMD_PAIR]) {
                    u8 links = hSamSub_NumClientsConnected();
                    // If we have all expected clients, just exit
                    if (3 == links) {
                        hSamSub_pairWanted = FALSE;
                        break;
                    }
                    hSamSub_pairCmdPresent = TRUE;  // Block repeated pairing from hSamSub_syncRegs()
                    // We have none of the clients linked
                    if (!links) {
                        // Look for one client below, plus two more
                        hSamSub_pairFindNextClient = 2;
                    }
                    // We have some of the clients linked
                    else {
                        // Look for one client below, plus zero more
                        hSamSub_pairFindNextClient = 0;
                        if (1 == links) {
                            // Look for one client below, plus one more
                            hSamSub_pairFindNextClient = 1;
                        }
                    }
                    hSamSub_pairTimeout = MANUAL_PAIRING_TIMEOUT_SECS;
                    hSamSub_pair();
                }
                else {
                    // Handle secondary auto-pairing
                    if (cmdregs[SAMSUB_TWIREG_CMD_MODE] != SAMSUB_MODE_READY &&
                        !hSamSub_autoPairingDone) {
                        hSamSub_autoPairingDone = TRUE;
                        // If we haven't learned all expected clients, we auto-pair...
                        if (pair_cli0_is_zero || pair_cli1_is_zero || pair_cli2_is_zero) {
                            u8 missing_clients = 0;
                            if (pair_cli1_is_zero) {
                                missing_clients++;
                            }
                            if (pair_cli2_is_zero) {
                                missing_clients++;
                            }
                            hSamSub_pairFindNextClient = missing_clients;
                            hSamSub_pairTimeout = SECOND_AUTO_PAIRING_TIMEOUT_SECS;
                            hSamSub_pair();
                        }
                    }
                }
                break;

            default:
                break;
            }
        }
    }

    // Trap the set of clients we are going to pair with
    // We need to store this to determine
    // when we have linked all clients solidly.
    if (hSamSub_doAutoPairing && doing_primary_auto_pairing) {
        hSamSub_doAutoPairing_appToken = romCfgCtxt.appToken;
    }

#if (CONFIG_SUPPORT_RF_FUNCTIONS)
    // Set antenna override, if needed (for debug only).
    if (stwis_regs[SAMSUB_TWIREG_RFTEST_ANTENNA] != hSamSub_RfTestAntenna) {
        stwis_regs[SAMSUB_TWIREG_RFTEST_ANTENNA] = appCmn_RfTestSetAntenna(stwis_regs[SAMSUB_TWIREG_RFTEST_ANTENNA]);
        hSamSub_RfTestAntenna = stwis_regs[SAMSUB_TWIREG_RFTEST_ANTENNA];
    }

    // Set Tx digital level, if needed (for debug only).
    if (stwis_regs[SAMSUB_TWIREG_TX_DIG_LEVEL] != hSamSub_RfTestTxDigLevel) {
        stwis_regs[SAMSUB_TWIREG_TX_DIG_LEVEL] = appCmn_RfTestSetTxDigLevel(stwis_regs[SAMSUB_TWIREG_TX_DIG_LEVEL]);
        hSamSub_RfTestTxDigLevel = stwis_regs[SAMSUB_TWIREG_TX_DIG_LEVEL];
    }
#endif

    /* Test all the registers that get forwarded to the client. */
    cr = &samSub_cliRegs[0];

    otasiSendsPerThread = 0;

    for (;;) {
        twiReg = cr->twiReg;

        if (twiReg == 0) { /* End of list. */
            break;
        }

        newVal = stwis_regs[twiReg];
        if ((cr->state == SAMSUB_REGSTATE_DIRTY) ||
            ((cr->state == SAMSUB_REGSTATE_UNSET) && stwis_getRegState(twiReg)) ||
            (newVal != cr->val)) {

            if (g_otasiAtHiWater)
            {
                /* OTASI is plenty busy and we don't want to run out of buffers.  We'll try again later. */
                stwis_retryLater(STWIS_MS_PER_RETRY);
                break;
            }
            else if (otasiSendsPerThread >= 3) // No need to queue up too much and overuse buffs.
            {
                stwis_retryLater(STWIS_MS_PER_RETRY);
                break;
            }

            if (!appIsActiveLinks()) {
                /* Give up, network is down. But the system needs to check standby mode.
                   Do not return from here. Samsung wants to use fake standby and deep standby
                   by setting standby register 1 for fake standby and 2 for deep standby.
                   RM #5691 fake standby -> deep standby by 0x20 register.
                   In deep standby mode, all network connection will be disconnected.
                   If we return here, the below standby mode code will not be reached.
                 */
                /* Restored from CL #261386 - Just use previous standby mode until Samsung
                   validates system level functionality
                */
                return;
            }

            if ((SAMSUB_OTASI_SUB == cr->tgt) || (SAMSUB_OTASI_ALL == cr->tgt)) {
                if (hSamSub_sendDataToClient(CmdNodeId_Listener_1, twiReg, newVal))
                    otasiSendsPerThread++ ;
            }

            if ((SAMSUB_OTASI_REAR == cr->tgt) || (SAMSUB_OTASI_ALL == cr->tgt)) {
                if (twiReg != SAMSUB_TWIREG_SPEAKER_MUTE || (newVal & 0xF0) == 0) {
                    if (hSamSub_sendDataToClient(CmdNodeId_Listener_2, twiReg, newVal))
                        otasiSendsPerThread++ ;
                    if (hSamSub_sendDataToClient(CmdNodeId_Listener_3, twiReg, newVal))
                        otasiSendsPerThread++ ;
                }
            }
            else if (SAMSUB_OTASI_LREAR == cr->tgt) {
                if (hSamSub_sendDataToClient(CmdNodeId_Listener_2, twiReg, newVal))
                    otasiSendsPerThread++ ;
            }
            else if (SAMSUB_OTASI_RREAR == cr->tgt) {
                if (hSamSub_sendDataToClient(CmdNodeId_Listener_3, twiReg, newVal))
                    otasiSendsPerThread++ ;
            }

            // Special case handling
            if (twiReg != SAMSUB_TWIREG_FAKE_STANDBY)
            {
                cr->state = SAMSUB_REGSTATE_CLEAN;
                cr->val = newVal;
            }
            else if (newVal == hSamSub_FakeStbyState)
            {
                u8 clients;
                clients = (stwis_regs[SAMSUB_TWIREG_LINK_STATUS] == 7 ? 7 : (stwis_regs[SAMSUB_TWIREG_LINK_STATUS] ^ 7) ) & 0x07;

                if ((hSamSub_FakeStbyState_acks & clients) == clients)
                {
                    cr->state = SAMSUB_REGSTATE_CLEAN;
                    cr->val = newVal;
                }
                else
                {
                    stwis_retryLater(STWIS_MS_PER_RETRY*2); // a little longer for round trip
                }
            }
            else
            {
                stwis_retryLater(STWIS_MS_PER_RETRY*2);
            }
        }
        ++cr;
    }

    // If enabled, this will break the link to the client(s). Do this AFTER
    // the above code has synchronized with them client(s)...
    if (stwis_regs[SAMSUB_TWIREG_FAKE_STANDBY] != hSamSub_FakeStbyState) {
        u8 hSamSub_FakeStbyState_prev = hSamSub_FakeStbyState;
        hSamSub_FakeStbyState = stwis_regs[SAMSUB_TWIREG_FAKE_STANDBY];
        hSamSub_FakeStbyState_acks = 0;
        samSub_cliRegs[0].state = SAMSUB_REGSTATE_DIRTY;
        stwis_retryLater(STWIS_MS_PER_RETRY*2);

        // We need to make sure pairing is cancelled before entering standby.
        if (stwis_regs[SAMSUB_TWIREG_FAKE_STANDBY]) {
            appCancelPairing();
        }

        /* Restore previous link by using MAC search */
        if (2 == hSamSub_FakeStbyState_prev && 0 == hSamSub_FakeStbyState) {
            macSearch();
        }
        /* Turn off RF and disconnect link... */
        else if (2 == hSamSub_FakeStbyState) {
            macStandby();
        }
    }
}

bool hSamSub_nodeIdIsPaired (CmdNodeId node_id) {
    if ((CmdNodeId_Listener_1 == node_id) && !pair_cli0_is_zero) {
        return TRUE;
    }

    if ((CmdNodeId_Listener_2 == node_id) && !pair_cli1_is_zero) {
        return TRUE;
    }

    if ((CmdNodeId_Listener_3 == node_id) && !pair_cli2_is_zero) {
        return TRUE;
    }

    return FALSE;
}

bool hSamSub_findClient(CmdNodeId node_id) {
    u8 i;
    bool found = FALSE;

    // See if the client this message is intended for is connected and paired (belt and suspenders)
    for (i = 0; i < romCfgCtxt.maxNumLinks; i++) {
        if (((macCtxtData.link[i].state == MacSlotState_Active)
            && peerCache[macCtxtData.link[i].peerCacheIdx].netNodeId == node_id)
            && (hSamSub_nodeIdIsPaired(node_id)))
        {
            found = TRUE;
            break;
        }
    }

    return found;
}

bool hSamSub_sendDataToClient(CmdNodeId node_id, u8 twiReg, u8 newVal) {
    CmdQueueEntry *cmd;
    CmdBuffer *buf;

    // If the intended client isn't found, don't bother sending data OTA...
    if (FALSE == hSamSub_findClient(node_id)) {
        return FALSE;
    }

    cmd = cmdGet(CmdModuleId_CmdLoop);
    cmd->buf.ext.opcode = (twiReg == SAMSUB_TWIREG_FAKE_STANDBY && ((hSamSub_FakeStbyState_acks & (1 << (node_id-1))) == 0)
                       ? CmdCode_Extended_Query
                       : CmdCode_Extended_Command);
    cmd->buf.ext.src = CmdModuleId_App;
    cmd->buf.ext.dst = CmdModuleId_App;
    cmd->buf.ext.node = CmdRouteSetNodeId(CmdNodeId_Source, node_id);
    buf = cmdBufferGet_LT();
    cmd->pBuffer = buf;
    buf->buffer[0] = 4; /* Length. */
    buf->buffer[1] = SAMSUB_SUBCMDE_SET_REG;
    buf->buffer[2] = twiReg;
    buf->buffer[3] = newVal;
    cmdSendCommand_LT(cmd, CmdQueuePriority_LOW);

    return TRUE;
}

// Set TX digital atenuation on a Rear Right client
void hSamSub_setRearRightSpk_tx_power(void) {
    CmdQueueEntry *cmd;
    CmdBuffer *buf;

    cmd = cmdGet(CmdModuleId_CmdLoop);
    cmd->buf.ext.opcode = CmdCode_Extended_Command;
    cmd->buf.ext.src = CmdModuleId_App; //CmdModuleId_App; CmdModuleId_TWIS;
    cmd->buf.ext.dst = CmdModuleId_CmdLoop; //CmdModuleId_CmdLoop; CmdModuleId_App;
    cmd->buf.ext.node = CmdRouteSetNodeId(CmdNodeId_Source, CmdNodeId_Listener_3);  //Rear Right
    buf = cmdBufferGet_LT();
    cmd->pBuffer = buf;
    buf->buffer[0] = 5; /* Length. */
    buf->buffer[1] = LoopCommand_Poke;
    buf->buffer[2] = 0xC3;  //reg. TX_DIG_LEVEL (address = 0xF3C3)
    buf->buffer[3] = 0xF3;
    buf->buffer[4] = 27;    //attenuation level
    cmdSendCommand_LT(cmd, CmdQueuePriority_LOW);
}

const u8 linkStatBits[MAC_LINK_CONTEXTS] = {CONFIG_SAMSUB_REARLEFT_LINKSTATUS_BIT, CONFIG_SAMSUB_SUBW_LINKSTATUS_BIT, CONFIG_SAMSUB_REARRIGHT_LINKSTATUS_BIT};

#define MAX_CLIENT_DROPS_DURING_APAIR 2
void hSamSub_updateLinkStatusReg(void) {
    u8 i;
    static u8 linkStatus, currAppToken;
    static u8 prevLinkstatus;
    static bool prevState, currState;
    bool droppedSomeClients = FALSE;
    static u8 clientDropCount[3] = {0, 0, 0};

    linkStatus = 0;

    // Log state here only.  App token updates are moved to the pair success
    // link state notifcation section of the app command handler above.

    if (hSamSub_findClient(CmdNodeId_Listener_1)) {         // sub
        linkStatus |= CONFIG_SAMSUB_SUBW_LINKSTATUS_BIT;
    }
    if (hSamSub_findClient(CmdNodeId_Listener_2)) {         // rearLeft
        linkStatus |= CONFIG_SAMSUB_REARLEFT_LINKSTATUS_BIT;
    }
    if (hSamSub_findClient(CmdNodeId_Listener_3)) {         // rearRight
        linkStatus |= CONFIG_SAMSUB_REARRIGHT_LINKSTATUS_BIT;
    }

    prevLinkstatus = stwis_regs[SAMSUB_TWIREG_LINK_STATUS];
    stwis_regs[SAMSUB_TWIREG_LINK_STATUS] = linkStatus;
    currAppToken = romCfgCtxt.appToken;

    if (doing_primary_auto_pairing) {

        // We are in auto pairing mode. Check if we've dropped a
        // connection. In which case we'll go into search-pair-search to try
        // and bring all clients in

        for (i = 0; i < romCfgCtxt.maxNumLinks; ++i) {
            prevState = (((prevLinkstatus | ((~currAppToken) & hSamSub_doAutoPairing_appToken)) & linkStatBits[i]) != 0);
            currState = ((linkStatus & linkStatBits[i]) != 0);

            if ( !prevState && currState) {
                // A client went from disconnected to connected;
            }

            // We have detected a dropped client during autopairing
            if ( prevState && !currState ) {
                hSamSub_autoPairClientDropped = TRUE;

                // Fix up the token for the dropped client
                if ((i == 1) && (clientDropCount[i]++ < MAX_CLIENT_DROPS_DURING_APAIR)) {
                    romCfgCtxt.appToken |= (
                            SAMSUB_MACTOKEN_SUBWOOFER);
                    droppedSomeClients = TRUE;
                }
                // Allow rear clients in  manual pair mode
                if ((i == 0) && (clientDropCount[i]++ < MAX_CLIENT_DROPS_DURING_APAIR)) {
                    romCfgCtxt.appToken |= (
                            SAMSUB_MACTOKEN_REARLEFT);
                    droppedSomeClients = TRUE;
                }
                if ((i == 2) && (clientDropCount[i]++ < MAX_CLIENT_DROPS_DURING_APAIR)) {
                    romCfgCtxt.appToken |= (
                            SAMSUB_MACTOKEN_REARRIGHT);
                    droppedSomeClients = TRUE;
                }
            }
        }

        if (droppedSomeClients && (!hSamSub_pairStepPending))
        {
            hSamSub_pairStepPending = TRUE;
            cmdSendCommand_LT(&hSamSub_pairStepCmd, CmdQueuePriority_LOW);
        }

        // If state is active and not active_pair/search (max clients connected) and the max
        // drop counts have been reached, terminate the retry stuff but not the autopairing unless we are active and
        // all clients have bounced the max.

        if (( MacCtxtState_Active == hSamSub_curState) &&
            ((clientDropCount[0] > MAX_CLIENT_DROPS_DURING_APAIR)
            && (clientDropCount[1] > MAX_CLIENT_DROPS_DURING_APAIR)
            && (clientDropCount[2] > MAX_CLIENT_DROPS_DURING_APAIR))) {

            hSamSub_endAutoPairCycle();
        }

        else if (romCfgCtxt.appToken == 0) {  // We have everyone for now, pause the retry system
            hSamSub_autoPairClientDropped = FALSE;
            appCancelPairing();
        }

    }
    else {
        // We are in manual "hSamSub_pairFindNextClient" mode
    }
}

// Called after radio enabled, but only valid once
// the MAC has completed WLAN scan, plus some time
// to enter searching...so if we aren't in that state already,
// we need to let the MAC get there before trying again.
bool hSamSub_letRadioStabilize(void) {
    if (FALSE == hSamSub_radioStabilized) {
        // Start a timer to retry checking the radio
        if (!hSamSub_radioStabilizedTimer) {
            u8 delay = 3;   // Delay 3 seconds if MacCtxtState_Search
            if ((hSamSub_curState == MacCtxtState_IScan)){
                delay = 6;  // Delay 6 seconds for MacCtxtState_IScan
            }
            hSamSub_radioStabilizedTimer = timerRequestSecond(FALSE, FALSE, (u16 *)&hSamSub_handleTwiWriteCmd, delay);
            ASSERT(hSamSub_radioStabilizedTimer);
        }
        return FALSE;
    }
    return TRUE;
}

u8 hSamSub_NumClientsConnected(void) {
    u8 i;
    u8 links = 0;

    for (i = 0; i < romCfgCtxt.maxNumLinks; i++) {
        if (macCtxtData.link[i].state == MacSlotState_Active) {
            links++;
        }
    }

    return links;
}

void hSamSub_handleStateChange(void) {
    u16 statetimer;

    if (appIsActiveLinks()) {
        if (!hSamSub_regUpdateScheduled) {
            hSamSub_regUpdateScheduled = TRUE;

            statetimer = timerRequestmSec(FALSE, FALSE, (u16 *)&hSamSub_syncRegsCmd, 250);
            ASSERT(statetimer);
        }
    }
}

void hSamSub_syncRegs(void) {
    SamSub_CliReg *cr;
    //u16 statetimer;

    if (macCtxtData.state != hSamSub_curState) {
        hSamSub_curState = macCtxtData.state;
    }

    if (appIsActiveLinks()) {
        /* Make sure that all state variables get sent to the client. */
        for (cr = &samSub_cliRegs[0]; cr->twiReg != 0; ++cr) {
            if (cr->state == SAMSUB_REGSTATE_CLEAN) {
                cr->state = SAMSUB_REGSTATE_DIRTY;
            }
        }

        // If we have an image update in progress, defer the OTA updating.
        if (!hUpdateImgTickle) {
            hSamSub_handleTwiWrite();
        }
        else {
            hUpdateImgTickle = FALSE;
        }
    }
}

void hSamSub_pair(void) {

    typedef enum AppMacPair_t {
       appMacPairUseNewRandID = 1
       ,appMacPairUseMyCurrentID = 2
    } AppMacPairType;
    AppMacPairType type;

    if (!SAMSUB_IS_HOST_FACTORY_TEST(myId.id.as_u32) && !hSamSub_tooEarlyToPair) {
        hSamSub_pairWanted = FALSE;

        // Don't use random ID unless no clients have been learned
        type =  (((pair_cli0_is_zero && pair_cli1_is_zero && pair_cli2_is_zero) &&
                    !hSamSub_isAlwaysPair)
                ? appMacPairUseNewRandID
                : appMacPairUseMyCurrentID);

        if (type == appMacPairUseNewRandID)
        {
            macRand(&myId.id.as_u32);
        }
        macPair();
        if ((type == appMacPairUseNewRandID) &&
            ((myId.id.as_u32 == 0) ||
             SAMSUB_IS_HOST_FACTORY_TEST(myId.id.as_u32) ||
             hSamSub_isAlwaysPair)) {
            myId.id.by_u8[3] ^= 0x1;
        }

        // Handle pairing timeout
        if (!SAMSUB_IS_ALWAYS_PAIR(myId.id.as_u32)) {
            cancelAndZeroTimer((u16 *)&hSamSub_pairTimer);
            hSamSub_pairTimer = timerRequestSecond(FALSE, TRUE, (u16 *)hSamSub_pairTimer_cb, hSamSub_pairTimeout);
            ASSERT(hSamSub_pairTimer);
        }
    }
}

void hSamSub_enableRadio(void) {
    if (!hSamSub_radioEnabled)
    {
        hSamSub_radioEnabled = TRUE;

        romCfgCtxt.enterStandbyPostScan = FALSE;

        macInitBand();
        macCqLTInit();
        macCqConfig();
        macHopSyncHop(MacHopSyncChReset);
        macGoPtm();
    }
}

/*! \brief Common code to cancel and zero out a active timer.
*/
void cancelAndZeroTimer(u16 * tmr)
{
    u8 intState;

    intState = intDisable_L1();
    if (0 != *tmr)
    {
        timerCancel_L1(*tmr);
        *tmr = 0;
    }
    intEnable_L1(intState);
}

/*! \brief Test for auto pair state.  Move from a MACRO to
 *    save memory since it is widely used
*/
bool hSamSub_IsAlwaysPair(u32 id)
{
    return(((id) & ~SAMSUB_PAIR_MASK) == SAMSUB_FACTORY_ALWAYS_PAIR_ID);
}

#if CARRIER
void toggleLedCallback(void)
{
    cGpio_toggle(CGPIO_BLUE_LED);
}
#endif

void appStartup(void)
{
    // Set or clear protection of flash boot sector and factory image.
    setFlashProtection(protectFlash);

    /// \todo Definitely room for improvement here in terms of memory. A
    /// lot of macPostInit could be moved into the init section.
    macPostInit();
    appCmnConfigPost();
    stwis_init();

    // Auto-pairing only done via SAMSUB_TWIREG_CMD_MODE register...
    //hSamSub_pairWanted = ((pairedCliId0.id.as_u32 == 0) || (pairedCliId1.id.as_u32 == 0));
    hSamSub_isAlwaysPair = SAMSUB_IS_ALWAYS_PAIR(myId.id.as_u32);
    if (hSamSub_isAlwaysPair) {
        // Set desired channel(s) in locked set
        hSamsub_fcty_setfixedFreq(myId.id.by_u8[0]);
    }

    samsub_cmdInit(&hSamSub_pairStepCmd, SAMSUB_SUBCMDS_PAIR_STEP);
    samsub_cmdInit(&hSamSub_timerpairStepCmd, SAMSUB_SUBCMDS_TIMERPAIR_STEP);

    samsub_cmdInit(&pairSm_searchNpair_stepCommand, SAMSUB_SUBCMDS_SEARCHNPAIR_STEP);
    samsub_cmdInit(&hSamSub_syncRegsCmd, SAMSUB_SUBCMDS_SYNC_REGS);
    samsub_cmdInit(&hSamSub_handleTwiWriteCmd, SAMSUB_SUBCMDS_TWI_WRITE);
//    samsub_cmdInit(&hSamSub_linkStatusUpdateCmd, SAMSUB_SUBCMDS_UPDATE_LINK_STATUS);

    hSamSub_pairTimer = timerRequestmSec(FALSE, TRUE, (u16 *)hSamSub_pairTimer_cb, 2000);
    ASSERT(hSamSub_pairTimer);

    hSamSub_pairTimer = timerRequestmSec(FALSE, FALSE, (u16 *)&hSamSub_pairStepCmd, 2000);
    ASSERT(hSamSub_pairTimer);

#if (CONFIG_SUPPORT_RF_FUNCTIONS)
    stwis_regs[SAMSUB_TWIREG_RFTEST_ANTENNA] = appCmn_RfTestSetAntenna(SAMSUB_RFTEST_ANTENNA_DIVERSITY);
    stwis_regs[SAMSUB_TWIREG_TX_DIG_LEVEL] = hSamSub_RfTestTxDigLevel = RegRead(PHY_CONFIG, TX_DIG_LEVEL_REG);
#endif

#if CARRIER
    {
    u16 togLegCallbTimer;
    // Enable a periodic "heartbeat" timer. We use a second timer,
    // rather than millisecond, because there are only 5 of each
    // available, and the millisecond timers seem to be used more.
    cGpio_config_output(CGPIO_BLUE_LED);
    togLegCallbTimer = timerRequestSecond(TRUE, TRUE, (u16 *)toggleLedCallback, 1);
    ASSERT(togLegCallbTimer);
    }
#endif

#if defined(MFCFG_HOST_REGTORY) || defined(MFCFG_HOST_REGTORY_FHSS) || defined(MFCFG_HOST_REGTORY_DSBEU)
    hSamSub_enableRadio();
#endif

    // This app supports 3 clients and therefore requires more buffers.
    // Add some buffers now.
    {
        u8 idx;

        for (idx = 0; idx < CMD_BUFFER_POOL_NUM_EXT_BUFFERS; idx++)
        {
            cmdBufferPut_LT(&(cmdBufferPoolExt[idx]));
        }
    }

    appMasterCommand();
}

