import sys
sys.path.append('/home/tim/Documents/MQP_ws/src')

from base_package.src import moving_demo
from abstract_arm.moveItClient import MoveitArmClient

class FullBaseDemo:
    def __init__(self):
        self.base_controller = moving_demo.MoveDemo()
        self.arm_controller = MoveitArmClient()


if __name__ == "__main__":
    f = FullBaseDemo()