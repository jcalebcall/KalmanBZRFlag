import numpy.matlib
import numpy as np
from numpy.linalg import inv
import matplotlib as plt

import sys
import math
import time

from bzrc import BZRC, Command

class KalmanAgent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.X = np.matlib.zeros((6, 1))
        self.u_t = np.matlib.zeros((6, 1))
        self.E_x = np.matrix([[.1, 0, 0, 0, 0, 0],
                             [0, .1, 0, 0, 0, 0,],
                             [0, 0, 50, 0, 0, 0],
                             [0, 0, 0, .1, 0, 0],
                             [0, 0, 0, 0, .1, 0],
                             [0, 0, 0, 0, 0, 50]])
        self.E_t = self.E_x
        self.t = .5
        t = self.t
        self.F = np.matrix([[1, t, pow(t,2)/2, 0, 0, 0],
                           [0, 1, t, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 1, t, pow(t,2)/2],
                           [0, 0, 0, 0, 1, t],
                           [0, 0, 0, 0, 0, 1]])
        self.H = np.matrix([[1, 0, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0]])
        self.E_z = np.matrix([[25, 0],
                             [0, 25]])
        self.prevTime = time.time()
        #self.estimates_x = np.()

        self.constants = self.bzrc.get_constants()
        self.commands = []

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]

        self.commands = []


        for i in range(1, len(self.mytanks)):
            self.kalman_filter(self.mytanks[i], self.othertanks[i])

        print(self.E_t)
        #results = self.bzrc.do_commands(self.commands)

    def kalman_filter(self, tank, othertank):
        # Kalman update equations
        z_t = np.matrix([othertank.x],
                        [othertank.y])

        F = self.F
        E_x = self.E_x
        H = self.H
        E_z = self.E_z
        E_t = self.E_t
        u_t = self.u_t
        temp = F * E_t * F.transpose() + E_x
        temp2 = inv(H * temp * H.transpose() + E_z)

        K_t_1 = temp * H.transpose() * temp2
        u_t_1 = F * E_t + K_t_1 * (z_t - H * F * u_t)
        E_t_1 = (np.identity(6) - K_t_1 * H) * temp

        self.E_t = E_t_1
        self.u_t = u_t_1

        # Reset variance matrix every 10 seconds
        if time.time() - self.prevTime > 10:
            self.E_t = self.E_x
            self.prevTime = time.time()

    #def plot_kalman(self):


    def attack_enemies(self, tank):
        """Find the closest enemy and chase it, shooting as you go."""
        best_enemy = None
        best_dist = 2 * float(self.constants['worldsize'])
        for enemy in self.enemies:
            if enemy.status != 'alive':
                continue
            dist = math.sqrt((enemy.x - tank.x)**2 + (enemy.y - tank.y)**2)
            if dist < best_dist:
                best_dist = dist
                best_enemy = enemy
        if best_enemy is None:
            command = Command(tank.index, 0, 0, False)
            self.commands.append(command)
        else:
            self.move_to_position(tank, best_enemy.x, best_enemy.y)

    def move_to_position(self, tank, target_x, target_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        command = Command(tank.index, 1, 2 * relative_angle, True)
        self.commands.append(command)

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle


def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    agent = KalmanAgent(bzrc)

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4
