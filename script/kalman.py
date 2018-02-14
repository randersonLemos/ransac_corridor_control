import numpy
import utils

class EKF(object):
    def __init__(self, l, x, P, Q, R, verbose):
        self.l = l
        self.x = x
        self.P = P
        self.Q = Q
        self.R = R
        self.verbose = verbose
        self.I = numpy.matrix(numpy.identity(6))

        print 'EKF parameters...'
        print 'Distance between axle car: ',self.l
        print 'Process covariance matrix: \n',self.Q
        print 'Measurement covariance matrix: \n', self.R

        print 'Initial guess...'
        print 'x: \n', self.x
        print 'P: \n', self.P


    def f(self, x, dt):
        be,b,om,al,v,a = utils.split_state(x)
        tan = numpy.tan
        return numpy.matrix([
                             [be - dt*om]
                            ,[b + dt**2*om*v + (-b*dt*om + dt*v)*tan(be - dt*om)]
                            ,[om+al*dt]
                            ,[al]
                            ,[v+a*dt]
                            ,[a]
                           ])

    def F(self, x, dt):
        be,b,om,al,v,a = utils.split_state(x)
        tan = numpy.tan
        return numpy.matrix([
                              [1.0, 0.0, -dt, 0.0, 0.0, 0.0]
                             ,[(-b*dt*om + dt*v)*(tan(be - dt*om)**2 + 1.0), -dt*om*tan(be - dt*om) + 1.0, -b*dt*tan(be - dt*om) + dt**2*v - dt*(-b*dt*om + dt*v)*(tan(be - dt*om)**2 + 1.0), 0.0, dt**2*om + dt*tan(be - dt*om), 0.0]
                             ,[0.0, 0.0, 1.0, dt, 0.0, 0.0]
                             ,[0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
                             ,[0.0, 0.0, 0.0, 0.0, 1.0, dt]
                             ,[0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                            ])

    def h(self, x):
        be,b,om,al,v,a = utils.split_state(x)
        return numpy.matrix([
                             [be]
                            ,[b]
                            ,[om]
                            ,[v]
                           ])

    def H(self,x):
        return numpy.matrix([
                              [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                             ,[0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
                             ,[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
                             ,[0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
                            ])

    def predict(self, dt):
        if self.verbose:
            print 'predict'
            print 'dt',dt
            print 'x: \n', self.x
            print 'P: \n', self.P

        self.P = self.F(self.x, dt)*self.P*self.F(self.x, dt).transpose() + self.Q
        self.x = self.f(self.x, dt)
        if self.verbose:
            print 'x: \n', self.x
            print 'P: \n', self.P

    def update(self, z):
        if self.verbose:
            print 'update'
            print 'z: \n',z
            print 'x: \n',self.x
            print 'P: \n',self.P

        inov = z - self.h(self.x) # innovation
        if self.verbose:
            print 'inov: \n',inov

        S = self.H(self.x)*self.P*self.H(self.x).transpose() + self.R
        K = self.P*self.H(self.x).transpose()*numpy.linalg.inv(S)
        if self.verbose:
            print 'S: \n',S
            print 'inv(s): \n', numpy.linalg.inv(S)
            print 'K: \n',K

        self.P = (self.I - K*self.H(self.x))*self.P # updated covariance estimate
        self.x = self.x + K*inov # updated state estimate
        if self.verbose:
            print 'x: \n',self.x

    def get_state(self):
        return self.x

    def get_cov_matrix(self):
        return self.P
