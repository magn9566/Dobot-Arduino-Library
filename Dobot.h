/*
** Based on Dobot-Communication-Protocol-V1.1.5.pdf
**
** Copyright (c) 2022, Magnus Weidemann
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#ifndef DOBOT_H
#define DOBOT_H

#include <Arduino.h>
#include <HardwareSerial.h>

#define DOBOT_SYNC_BYTE 0xAA
#define DOBOT_MAX_PAYLOAD_SIZE (DOBOT_SYNC_BYTE - 1)

#define DOBOT_DEBUG_SERIAL Serial

#define DOBOT_DEBUG_LEVEL_NONE 0
#define DOBOT_DEBUG_LEVEL_ERRORS 2
#define DOBOT_DEBUG_LEVEL_WARNINGS 3
#define DOBOT_DEBUG_LEVEL_INFO 4
#define DOBOT_DEBUG_LEVEL_DEBUG 5
#define DOBOT_DEBUG_LEVEL_VERBOSE 6

#define DOBOT_DEBUG_LEVEL DOBOT_DEBUG_LEVEL_INFO

class Dobot
{
public:
    // inspired by : https://stackoverflow.com/questions/21295935/can-a-c-enum-class-have-methods
    // to allow cast to int

    class RETURN_CODE
    {
    public:
        enum Value : int
        {
            SUCCESS = 0,
            GENERAL_ERROR = -1,
            SIZE_ERROR = -2,
            TIMEOUT = -3,
            PARM_SIZE = -4,
        };
        RETURN_CODE() = default;
        constexpr RETURN_CODE(Value aCode) : value(aCode) {}

        // Allow switch and comparisons.
        constexpr operator Value() const { return value; }

        // Prevent usage: if(RETURN_CODES)
        // explicit operator bool() const = delete;

        constexpr bool operator==(RETURN_CODE a) const { return value == a.value; }
        constexpr bool operator!=(RETURN_CODE a) const { return value != a.value; }
        constexpr bool operator<=(RETURN_CODE a) const { return value <= a.value; }
        constexpr bool operator>=(RETURN_CODE a) const { return value >= a.value; }

    private:
        Value value;
    };

    Dobot(HardwareSerial *serial);

    ~Dobot();

    void begin();
    void end();

    RETURN_CODE getSerialNumber(char *sn, size_t max_length);

    /// @brief Get stateus of the 8 alarms.
    /// @details alarm codes in dobot-magician-alarm-en.pdf
    /// @param alarms List to store the 8 alarm status in
    /// @return 0 on success else error code
    RETURN_CODE getAlarmState(int (*alarms)[8]);

    /// @brief Clear alarm states
    /// @return 0 on success else error code
    RETURN_CODE clearAlarmState();

    /// @brief This command is to set homing position.
    /// @details The default home position is (0°, 45°,45°, 0°).
    /// @param queued if true then command is queued with the returned index.
    /// @param x new x coordinate.
    /// @param y new y coordinate.
    /// @param z new z coordinate.
    /// @param r new r coordinate.
    /// @param queuedCmdIndex will be updated with queue index if given, ignored when command is not queued.
    /// @return RETURN_CODE::SUCCESS on success else error code
    RETURN_CODE setHomeParams(bool queued, float x, float y, float z, float r, uint64_t *queuedCmdIndex = NULL);

    /// @brief This command is to get current homing position.
    /// @details The default home position is (0°, 45°,45°, 0°).
    /// @param x returns home x coordinate.
    /// @param y returns home y coordinate.
    /// @param z returns home z coordinate.
    /// @param r returns home r coordinate.
    /// @return RETURN_CODE::SUCCESS on success else error code
    RETURN_CODE getHomeParams(float *x, float *y, float *z, float *r);

    /// @brief This command is to execute the homing function.
    /// @param queued if true then command is queued with the returned index.
    /// @param queuedCmdIndex will be updated with queue index if given, ignored when command is not queued.
    /// @return RETURN_CODE::SUCCESS on success else error code
    RETURN_CODE setHomeCmd(bool queued, uint64_t *queuedCmdIndex = NULL);

    RETURN_CODE setEndEffectorParams(bool queued, float xBias, float yBias, float zBias, uint64_t *queuedCmdIndex = NULL);
    RETURN_CODE getEndEffectorParams(float xBias, float yBias, float zBias);

    RETURN_CODE setEndEffectorSuctionCup(bool queued, bool isCtrlEnabled, bool isSucked, uint64_t *queuedCmdIndex = NULL);
    RETURN_CODE getEndEffectorSuctionCup(bool *isCtrlEnabled, bool *isSucked);
    RETURN_CODE setPTPCmd(bool queued, uint8_t PTPMode, float x, float y, float z, float r, uint64_t *queuedCmdIndex = NULL);
    RETURN_CODE debugInfo();

private:
    class ProtocolId
    {
    public:
        enum Value : uint8_t
        {
            // Device information
            FunctionDeviceInfoBase = 0,
            DeviceSN = FunctionDeviceInfoBase + 0,
            DeviceName = FunctionDeviceInfoBase + 1,
            DeviceVersion = FunctionDeviceInfoBase + 2,

            // Pose
            FunctionPoseBase = 10,
            GetPose = FunctionPoseBase + 0,
            ResetPose = FunctionPoseBase + 1,
            GetKinematics = FunctionPoseBase + 2,

            // Alarm
            FunctionALARMBase = 20,
            AlarmsState = FunctionALARMBase + 0,
            ClearAllAlarmsState = FunctionALARMBase + 1,

            // HOME
            FunctionHOMEBase = 30,
            HOMEParams = FunctionHOMEBase + 0,
            HOMECmd = FunctionHOMEBase + 1,

            // HHT
            FunctionHHTBase = 40,

            HHTTrigMode = FunctionHHTBase + 0,
            HHTTrigOutputEnabled = FunctionHHTBase + 1,
            HHTTrigOutput = FunctionHHTBase + 2,

            // Function-Arm Orientation
            FunctionArmOrientationBase = 50,
            ArmOrientation = FunctionArmOrientationBase + 0,

            // End effector
            FunctionEndEffectorBase = 60,
            EndEffectorParams = FunctionEndEffectorBase + 0,
            EndEffectorLaser = FunctionEndEffectorBase + 1,
            EndEffectorSuctionCup = FunctionEndEffectorBase + 2,
            EndEffectorGripper = FunctionEndEffectorBase + 3,

            // Function-JOG
            FunctionJOGBase = 70,
            JOGJointParams = FunctionJOGBase + 0,
            JOGCoordinateParams = FunctionJOGBase + 1,
            JOGCommonParams = FunctionJOGBase + 2,
            JOGCmd = FunctionJOGBase + 3,

            // Function-PTP
            FunctionPTPBase = 80,

            PTPJointParams = FunctionPTPBase + 0,
            PTPCoordinateParams = FunctionPTPBase + 1,
            PTPJumpParams = FunctionPTPBase + 2,
            PTPCommonParams = FunctionPTPBase + 3,
            PTPCmd = FunctionPTPBase + 4,

            // Function-CP
            FunctionCPBase = 90,

            CPParams = FunctionCPBase + 0,
            CPCmd = FunctionCPBase + 1,

            // Function-ARC
            FunctionARCBase = 100,

            ARCParams = FunctionARCBase + 0,
            ARCCmd = FunctionARCBase + 1,

            // Function-WAIT
            FunctionWAITBase = 110,
            WAITCmd = FunctionWAITBase + 0,

            // Function-TRIG
            FunctionTRIGBase = 120,
            TRIGCmd = FunctionTRIGBase + 0,

            // Function-EIO
            FunctionEIOBase = 130,

            IOMultiplexing = FunctionEIOBase + 0,
            IODO = FunctionEIOBase + 1,
            IOPWM = FunctionEIOBase + 2,
            IODI = FunctionEIOBase + 3,
            IOADC = FunctionEIOBase + 4,

            // Function-CAL
            FunctionCALBase = 140,
            AngleSensorStaticError = FunctionCALBase + 0,

            // Function-WIFI
            FunctionWIFIBase = 150,
            WIFIConfigMode = FunctionWIFIBase + 0,
            WIFISSID = FunctionWIFIBase + 1,
            WIFIPassword = FunctionWIFIBase + 2,
            WIFIIPAddress = FunctionWIFIBase + 3,
            WIFINetmask = FunctionWIFIBase + 4,
            WIFIGateway = FunctionWIFIBase + 5,
            WIFIDNS = FunctionWIFIBase + 6,
            WIFIConnectStatus = FunctionWIFIBase + 7,

            // Function-TEST
            TESTBase = 220,
            UserParams = TESTBase + 0,

            // Function-ZDF
            ZDFBase = 230,
            ZDFCalibRequest = ZDFBase + 0,
            ZDFCalibStatus = ZDFBase + 1,

            // Function-QueuedCmd
            FunctionQueuedCmdBase = 240,
            QueuedCmdStartExec = FunctionQueuedCmdBase + 0,
            QueuedCmdStopExec = FunctionQueuedCmdBase + 1,
            QueuedCmdForceStopExec = FunctionQueuedCmdBase + 2,
            QueuedCmdStartDownload = FunctionQueuedCmdBase + 3,
            QueuedCmdStopDownload = FunctionQueuedCmdBase + 4,
            QueuedCmdClear = FunctionQueuedCmdBase + 5,
            QueuedCmdCurrentIndex = FunctionQueuedCmdBase + 6,
            QueuedCmdLeftSpace = FunctionQueuedCmdBase + 7,

            Max = 0xFF
        };

        ProtocolId() = default;
        constexpr ProtocolId(Value aCode) : value(aCode) {}

        // Allow switch and comparisons.
        constexpr operator Value() const { return value; }

        // Prevent usage: if(ProtocolId)
        explicit operator bool() const = delete;

        constexpr bool operator==(ProtocolId a) const { return value == a.value; }
        constexpr bool operator!=(ProtocolId a) const { return value != a.value; }
        constexpr bool operator<=(ProtocolId a) const { return value <= a.value; }
        constexpr bool operator>=(ProtocolId a) const { return value >= a.value; }

    private:
        Value value;
    };

// ensure 1 byte aligment in communication protol
#pragma pack(push)
#pragma pack(1)
    typedef struct HomeParams_t
    {
        float x; // Dobot coordinates X;
        float y; // Dobot coordinates y;
        float z; // Dobot coordinates z;
        float r; // Dobot coordinates r;
    };

    typedef struct HomeCmdParms_t
    {
        uint32_t reserved; // Reserved for future use
    };

    typedef struct Alarms_t
    {
        int16_t alarm[8];
    };

    typedef struct SerialNumberParms_t
    {
        uint8_t serialNumber[20]; // actual max length unknown but seen 11
    };

    typedef struct SuctionCupParms_t
    {
        uint8_t isCtrlEnable;
        uint8_t isSucked;
    };

    typedef struct PTPCmd_t
    {
        uint8_t ptpMode;
        float x;
        float y;
        float z;
        float r;
    };

    typedef uint64_t QueuedCmdIndex_t;

    struct Packet
    {
        uint8_t Header[2]; // Sync field
        uint8_t Len;       // length of payload = 2 + parm length.
        struct
        {
            uint8_t ID; // Protocol ID of request

            struct
            {
                bool rw : 1;       // 1 bit field : is it a write request?
                bool isQueued : 1; // 1 bit field : is command queued at the Dobot?
            } Ctrl;

            union
            {
                SerialNumberParms_t SerialNumberParms;
                SuctionCupParms_t SuctionCupParms;
                PTPCmd_t PTPCmd;
                Alarms_t Alarms;
                HomeParams_t HomeParams;
                HomeCmdParms_t HomeCmdParms;
                QueuedCmdIndex_t QueuedCmdIndex;
            } Params;
        } Payload;

        uint8_t Checksum; // reserve space for checksum, but location is based on Len as packet length is dynamic
    };
#pragma pack(pop)

    HardwareSerial *_serial;

    Packet rx, tx; // transmission buffers

    void print_packet(Packet *packet, const char *prefix);

    RETURN_CODE encode_packet(Packet *packet, ProtocolId id, bool rw, bool isQueued, uint8_t parm_length = 0, void *parms = NULL);

    RETURN_CODE transmit_packet(Packet *packet);
    RETURN_CODE receive_packet(Packet *packet, long timeout);
};

#endif
