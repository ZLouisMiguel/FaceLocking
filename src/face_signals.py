import cv2


def format_motor_status(command):
    """Return the compact motor status string used by the camera overlay."""
    connection = "ONLINE" if command.available else "OFFLINE"
    return f"MOTOR: {command.direction} {command.angle} deg [{connection}]"


class OutputSignalManager:
    SEARCHING = "SEARCHING"
    LOCKED = "LOCKED"
    LOST = "LOST"

    def __init__(self):
        self.state = self.SEARCHING
        self.status = "SEARCHING / UNLOCKED"
        self.color = (0, 0, 255)

    def update_status(self, is_locked, identity_name=None, state=None):
        if state is None:
            state = self.LOCKED if is_locked else self.SEARCHING

        self.state = state
        if state == self.LOCKED and is_locked:
            self.status = f"LOCKED: {identity_name or 'Unknown'}"
            self.color = (0, 255, 0)
        elif state == self.LOST:
            self.status = f"TARGET LOST: {identity_name or 'Unknown'}"
            self.color = (0, 165, 255)
        else:
            self.status = "SEARCHING / UNLOCKED"
            self.color = (0, 0, 255)

    def draw_overlay(self, frame, box=None):
        if box is not None:
            x, y, w, h = box
            cv2.rectangle(frame, (x, y), (x + w, y + h), self.color, 2)
            cv2.putText(
                frame,
                self.status,
                (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                self.color,
                2,
            )

        cv2.putText(
            frame,
            f"System Status: {self.status}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            self.color,
            2,
        )
        return frame
