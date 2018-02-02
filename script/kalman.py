import numpy
import utils

class EKF(object):
    def __init__(self, l, x, P, Q, R):
        self.l = l
        self.x = x
        self.P = P
        self.Q = Q
        self.R = R
        self.I = numpy.matrix(numpy.identity(4))


        self.R_line = self.R[0:2,0:2]
        self.R_car = self.R[2:4,2:4]

        print 'EKF parameters...'
        print 'Distance between axle car: ',self.l
        print 'Process covariance matrix: \n',self.Q
        print 'Line measurement covariance matrix: \n', self.R_line
        print 'Car measurement covariance matrix: \n', self.R_car

    def f(self, x, dt):
        """
        Non-linear model function
        """

        be,b,om,v = utils.split_state(x)

        return  numpy.matrix([
                              [be - dt*om]
                             ,[b + numpy.tan(be - dt*om)*(dt*(b*om + v))]
                             ,[om]
                             ,[v]
                            ])

    def F(self, x, dt):
        """
        Jacobian matrix of the non-linear model
        """

        be,b,om,v = utils.split_state(x)

        el10 = -dt*(b*om + v)*(-numpy.tan(dt*om - be)**2-1.0)
        el11 = -dt*numpy.tan(dt*om - be) + 1.0
        el12 = -dt*b*numpy.tan(dt*om - be) - dt**2*(b*om + v)*(numpy.tan(dt*om-be)**2+1.0)
        el13 = -dt*numpy.tan(dt*om - be)

        return numpy.matrix([
                              [1.0, 0.0, -dt, 0.0]
                             ,[el10, el11, el12, el13]
                             ,[0.0, 0.0, 1.0, 0.0]
                             ,[0.0, 0.0, 0.0, 1.0]
                           ])

    def h_line(self,x):
        """
        matrix of the measurement function
        """

        be,b,om,v = [el.item() for el in x]

        return numpy.matrix([
                             [be]
                            ,[b]
                          ])

    def h_car(self,x):
        """
        matrix of the measurement function
        """

        be,b,om,v = [el.item() for el in x]

        return numpy.matrix([
                            [om]
                           ,[v]
                          ])

    def H_line(self,x):
       """
       Jacobian matrix of the measurement function
       """

       return numpy.matrix([
                            [1.0, 0.0, 0.0, 0.0]
                           ,[0.0, 1.0, 0.0, 0.0]
                          ])



    def H_car(self,x):
       """
       Jacobian matrix of the measurement function
       """

       return numpy.matrix([
                            [0.0, 0.0, 1.0, 0.0]
                           ,[0.0, 0.0, 0.0, 1.0]
                          ])


    def predict(self, dt):
        print 'predict'
        print 'dt',dt
        self.x = self.f(self.x, dt)
        self.P = self.F(self.x, dt)*self.P*self.F(self.x, dt).T + self.Q

    def update_line(self, z):
        print 'update_line'
        print 'z: \n',z
        print 'x: \n',self.x

        inov = z - self.h_line(self.x) # innovation
        print 'inov: \n',inov
        S = self.H_line(self.x)*self.P*self.H_line(self.x).transpose() + self.R_line
        K = self.P*self.H_line(self.x).transpose()*numpy.linalg.inv(S)
        print 'S: \n',S
        print 'K: \n',K
        self.P = (self.I - K*self.H_line(self.x))*self.P # updated covariance estimate
        print 'K*inov: \n',K*inov
        self.x = self.x + K*inov # updated state estimate
        self.x[0:2,0] = z
        print 'x: \n',self.x

    def update_car(self, z):
        print 'update_car'
        print 'z: \n',z
        print 'x: \n',self.x

        inov = z - self.h_car(self.x) # innovation
        print 'inov: \n',inov
        S = self.H_car(self.x)*self.P*self.H_car(self.x).transpose() + self.R_car
        K = self.P*self.H_car(self.x).transpose()*numpy.linalg.inv(S)

        self.P = (self.I - K*self.H_car(self.x))*self.P # updated covariance estimate
        self.x = self.x + K*inov # updated state estimate
        print 'x: \n',self.x

    def get_state(self):
        return self.x

    def get_cov_matrix(self):
        return self.P
