from enum import IntEnum


class CommunicationProtocolIDs(IntEnum):

    GET_SET_DEVICE_SN = 0
    GET_SET_DEVICE_NAME = 1
    GET_POSE = 10
    RESET_POSE = 11
    GET_ALARMS_STATE = 20
    CLEAR_ALL_ALARMS_STATE = 21
    SET_GET_HOME_PARAMS = 30
    SET_HOME_CMD = 31
    SET_GET_HHTTRIG_MODE = 40
    SET_GET_HHTTRIG_OUTPUT_ENABLED = 41
    GET_HHTTRIG_OUTPUT = 42
    SET_GET_ARM_ORIENTATION = 50
    SET_GET_END_EFFECTOR_PARAMS = 60
    SET_GET_END_EFFECTOR_LAZER = 61
    SET_GET_END_EFFECTOR_SUCTION_CUP = 62
    SET_GET_END_EFFECTOR_GRIPPER = 63
    SET_GET_JOG_JOINT_PARAMS = 70
    SET_GET_JOG_COORDINATE_PARAMS = 71
    SET_GET_JOG_COMMON_PARAMS = 72
    SET_GET_PTP_JOINT_PARAMS = 80
    SET_GET_PTP_COORDINATE_PARAMS = 81
    SET_GET_PTP_JUMP_PARAMS = 82
    SET_GET_PTP_COMMON_PARAMS = 83
    SET_PTP_CMD = 84
    SET_CP_CMD = 91
    SET_QUEUED_CMD_START_EXEC = 240
    SET_QUEUED_CMD_STOP_EXEC = 241
    SET_QUEUED_CMD_CLEAR = 245
    GET_QUEUED_CMD_CURRENT_INDEX = 246
