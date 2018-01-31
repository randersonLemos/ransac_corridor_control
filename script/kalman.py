import numpy
import utils

class EKF(object):
    def __init__(self, dt, l, x, P, Q, R):
        self.dt = dt
        self.l = l
        self.x = x
        self.P = P
        self.Q = Q
        self.R = R
        self.I = numpy.matrix(numpy.identity(4))

    def f(self, x):
        """
        Non-linear model function
        """

        be,b,om,v = utils.split_state(x)

        return  numpy.matrix([
                              [be - self.dt*om]
                             ,[b + numpy.tan(be - self.dt*om)*(self.dt*(b*om + v))]
                             ,[om]
                             ,[v]
                            ])

    def F(self, x):
        """
        Jacobian matrix of the non-linear model
        """

        be,b,om,v = utils.split_state(x)

        el10 = -self.dt*(b*om + v)*(-numpy.tan(self.dt*om - be)**2-1.0)
        el11 = -self.dt*numpy.tan(self.dt*om - be) + 1.0
        el12 = -self.dt*b*numpy.tan(self.dt*om - be) - self.dt**2*(b*om + v)*(numpy.tan(self.dt*om-be)**2+1.0)
        el13 = -self.dt*numpy.tan(self.dt*om - be)

        return numpy.matrix([
                              [1.0, 0.0, -self.dt, 0.0]
                             ,[el10, el11, el12, el13]
                             ,[0.0, 0.0, 1.0, 0.0]
                             ,[0.0, 0.0, 0.0, 1.0]
                           ])

    def h(self,x):
        """
        Matrix of the measurement function
        """

        be,b,om,v = [el.item() for el in x]

        return numpy.matrix([
                             [be]
                            ,[b]
                            ,[om]
                            ,[v]
                          ])

    def H(self,x):
        """
        Jacobian matrix of the measurement function
        """

        return self.I

    def predict(self):
        self.x = self.f(self.x)
        self.P = self.F(self.x)*self.P*self.F(self.x).T + self.Q

    def update(self,z):
        print(z)
        inov = z - self.h(self.x) # innovation
        S = self.H(self.x)*self.P*self.H(self.x).transpose() + self.R
        K = self.P*self.H(self.x).transpose()*numpy.linalg.inv(S)

        self.P = (self.I - K*self.H(self.x))*self.P # updated covariance estimate
        self.x = self.x + K*inov # updated state estimate

    def get_state(self):
        return self.x

