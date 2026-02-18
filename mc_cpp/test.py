import roboticstoolbox as rtb
from spatialmath import SE3

def main():

    robot = rtb.models.Panda()

    print(robot)
    Te = robot.fkine(robot.qr)  # forward kinematics
    print(Te)

    Tep = SE3.Trans(0.6, -0.3, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
    sol = robot.ik_LM(Tep)         # solve IK
    print(sol)

    q_pickup = sol[0]
    print(robot.fkine(q_pickup))
    qt = rtb.jtraj(robot.qr, q_pickup, 50)
    robot.plot(qt.q, backend='pyplot', movie='panda1.gif')

    # env = robot.plot(robot.qz, backend="swift", block=False)
    # print("Swift started. If you are in Docker, expose the port and open the URL in your host browser.")
    # input("Press Enter to quit...\n")

    # robot.plot(robot.qz, backend="swift", block=False)
    # print("swift started")

if __name__ == "__main__":
    main()


