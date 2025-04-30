import utils
import threading
import time
import can
import struct
import queue

bus = utils.get_can_interface()
CAN_IDS = utils.parse_header_file()
CAN_IDS_NAME = {v: k for k, v in CAN_IDS.items()}

can_message_queue = queue.Queue()

def wait_for_can_message(can_ids: list[int], timeout: float = 30):
    deadline = time.time() + timeout

    while time.time() < deadline:
        try:
            msg = can_message_queue.get(timeout=timeout)

            if msg.arbitration_id in can_ids:
                return msg
        except queue.Empty:
            break
    
    print("TIMEOUT! DID NOT RECV CAN ids:", can_ids)
    quit()
    return

def can_read_thread():
    try:
        while True:
            for msg in bus:
                if msg.is_error_frame:
                    print("CAN ERROR:", msg)
                    continue

                if msg.arbitration_id == CAN_IDS["CANID_IO_STATUS"]:
                    tors, = struct.unpack(">H", msg.data)
                    # print(f"status: {tors:016b}")
                
                elif msg.arbitration_id == CAN_IDS["CANID_IO_ALIVE"]:
                    just_rebooted, = struct.unpack(">?", msg.data)
                    # print(f"alive: just_rebooted:{just_rebooted}")

                else:
                    frame_name = CAN_IDS_NAME.get(msg.arbitration_id, f"UNKNOWN_ID")
                    print("CAN > ", frame_name, msg)

                can_message_queue.put(msg)
    except can.exceptions.CanOperationError as e:
        print(e)

class IOBoard:
    def reboot():
        msg = can.Message(arbitration_id=CAN_IDS["CANID_IO_REBOOT"])
        bus.send(msg)

    def enable(state: bool):
        msg = can.Message(arbitration_id=CAN_IDS["CANID_IO_STEPPER_ENABLE"], data=[state])
        bus.send(msg)


    def home(stepper_id: int, max_relative_steps_before_error: int, tor_id: int, tor_state_to_end_homing: bool):
        data = bytearray(
            stepper_id.to_bytes(1) +
            max_relative_steps_before_error.to_bytes(2, signed=True) +
            tor_id.to_bytes(1) +
            tor_state_to_end_homing.to_bytes(1)
        )
        msg = can.Message(arbitration_id=CAN_IDS["CANID_IO_STEPPER_HOME"], data=data)
        bus.send(msg)

    def goto_abs(stepper_id: int, absolute_steps: int, acceleleration: int, max_velocity: int):
        data = bytearray(
            stepper_id.to_bytes(1) +
            absolute_steps.to_bytes(2, signed=True) +
            acceleleration.to_bytes(3) +
            max_velocity.to_bytes(2)
        )
        msg = can.Message(arbitration_id=CAN_IDS["CANID_IO_STEPPER_GOTO"], data=data)
        bus.send(msg)

def test_ioboard():
    STEPS_PER_REV = 200*8
    ACCEL = STEPS_PER_REV * 50
    MAX_VEL = STEPS_PER_REV * 3

    pos = 0

    print("========== TEST IO BOARD START ==========")

    print("test: CANID_IO_REBOOT")
    print("excepted: (nothing)")
    IOBoard.reboot()
    wait_for_can_message([CAN_IDS["CANID_IO_ALIVE"]])
    print("\n")
    time.sleep(1)

    print("test: DISABLE")
    print("excepted: (nothing)")
    IOBoard.enable(False)
    print("\n")
    time.sleep(1)

    print("test: ENABLE")
    print("excepted: (nothing)")
    IOBoard.enable(True)
    print("\n")
    time.sleep(1)

    print("test: GOTO with very low speed with very high accel (0 accel/decel step)")
    print("excepted: GOTO_FINISHED")
    IOBoard.goto_abs(4, STEPS_PER_REV//5, STEPS_PER_REV*60, STEPS_PER_REV//30)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: GOTO zero")
    print("excepted: GOTO_FINISHED")
    IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: GOTO zero")
    print("excepted: GOTO_FINISHED")
    IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: GOTO with very low negative speed with very high accel (0 accel/decel step)")
    print("excepted: GOTO_FINISHED")
    IOBoard.goto_abs(4, -STEPS_PER_REV//5, STEPS_PER_REV*60, STEPS_PER_REV//30)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: GOTO 3turn")
    print("excepted: GOTO_FINISHED")
    IOBoard.goto_abs(4, STEPS_PER_REV*3, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: GOTO -1turn")
    print("excepted: GOTO_FINISHED")
    IOBoard.goto_abs(4, -STEPS_PER_REV*3, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("(reset pos to zero)")
    IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: HOME 2turn")
    print("excepted: HOME_SUCCEEDED or HOME_FAILED")
    IOBoard.home(4, STEPS_PER_REV*2, 15, False)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_HOME_SUCCEEDED"], CAN_IDS["CANID_IO_STEPPER_HOME_FAILED"]])
    print("\n")
    time.sleep(1)

    print("test: HOME -2turn")
    print("excepted: HOME_SUCCEEDED or HOME_FAILED")
    IOBoard.home(4, -STEPS_PER_REV*2, 15, False)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_HOME_SUCCEEDED"], CAN_IDS["CANID_IO_STEPPER_HOME_FAILED"]])
    print("\n")
    time.sleep(1)

    print("(reset pos to zero)")
    IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: GOTO, sleep 1s, GOTO")
    print("excepted: ERROR_MOTION_IN_PROGRESS then GOTO_FINISHED")
    IOBoard.goto_abs(4, STEPS_PER_REV, ACCEL, MAX_VEL//5)
    time.sleep(1)
    IOBoard.goto_abs(4, -STEPS_PER_REV, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS"]])
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("(reset pos to zero)")
    IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: GOTO, sleep 1s, HOME")
    print("excepted: ERROR_MOTION_IN_PROGRESS then GOTO_FINISHED")
    IOBoard.goto_abs(4, -STEPS_PER_REV, ACCEL, MAX_VEL//5)
    time.sleep(1)
    IOBoard.home(4, STEPS_PER_REV, 15, False)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS"]])
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("(reset pos to zero)")
    IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: HOME, sleep 1s, GOTO")
    print("excepted: ERROR_MOTION_IN_PROGRESS then (HOME_FAILED or HOME_SUCCEEDED)")
    IOBoard.home(4, STEPS_PER_REV*2, 15, False)
    time.sleep(1)
    IOBoard.goto_abs(4, STEPS_PER_REV*2, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS"]])
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_HOME_SUCCEEDED"], CAN_IDS["CANID_IO_STEPPER_HOME_FAILED"]])
    print("\n")
    time.sleep(1)

    print("(reset pos to zero)")
    IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: DISABLE, sleep 1s, HOME")
    print("excepted: ERROR_NOT_ENABLED")
    IOBoard.enable(False)
    time.sleep(1)
    IOBoard.home(4, STEPS_PER_REV*2, 15, False)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_ERROR_NOT_ENABLED"]])
    print("\n")
    time.sleep(1)

    print("(reset pos to zero + enable)")
    IOBoard.enable(True)
    IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: DISABLE, sleep 500ms, GOTO")
    print("excepted: ERROR_NOT_ENABLED")
    IOBoard.enable(False)
    time.sleep(0.5)
    IOBoard.goto_abs(4, STEPS_PER_REV*3, ACCEL, MAX_VEL//50)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_ERROR_NOT_ENABLED"]])
    print("\n")
    time.sleep(1)

    print("(reset pos to zero + enable)")
    IOBoard.enable(True)
    IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: GOTO, sleep 1s, DISABLE")
    print("excepted: ERROR_DISABLED_DURING_MOTION")
    IOBoard.goto_abs(4, -STEPS_PER_REV*5, ACCEL//5, MAX_VEL//5)
    time.sleep(1)
    IOBoard.enable(False)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_ERROR_DISABLED_DURING_MOTION"]])
    print("\n")
    time.sleep(1)

    print("(reset pos to zero + enable)")
    IOBoard.enable(True)
    IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: HOME, sleep 1s, DISABLE")
    print("excepted: ERROR_DISABLED_DURING_MOTION")
    IOBoard.home(4, STEPS_PER_REV*5, 15, False)
    time.sleep(1)
    IOBoard.enable(False)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_ERROR_DISABLED_DURING_MOTION"]])
    print("\n")
    time.sleep(1)

    print("(reset pos to zero + enable)")
    IOBoard.enable(True)
    IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("test: GOTO 3turn")
    print("excepted: GOTO_FINISHED")
    IOBoard.goto_abs(4, STEPS_PER_REV*3, ACCEL, MAX_VEL)
    wait_for_can_message([CAN_IDS["CANID_IO_STEPPER_GOTO_FINISHED"]])
    print("\n")
    time.sleep(1)

    print("========== TEST IO BOARD END ==========")

def main():
    print("ALL KNOWN CAN IDENTIFIERS:")
    for key, value in CAN_IDS.items():
        print(f"{key}: {value:#04x}")

    print("starting read can thread...")
    thread_can_read = threading.Thread(target=can_read_thread)
    thread_can_read.start()

    test_ioboard()

    time.sleep(2)
    bus.shutdown()
    thread_can_read.join()

if __name__ == "__main__":
    main()