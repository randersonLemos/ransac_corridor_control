def fromThree2Two(coeffs):
  a = -float(coeffs[0]/(coeffs[1] + 1e-4)) # angular coefficient
  b = -float(coeffs[2]/(coeffs[1] + 1e-4)) # linear coefficient
  return (a,b)


def fromTwo2Three(coeffs):
  a = -float(coeffs[0])
  b = 1.0
  c = -float(coeffs[1])
  return (a,b,c)


def points_from_coeffs2(coeffs,npts,scale):
    a = coeffs[0]
    b = coeffs[1]

    lst = []
    # for i in xrange(npts):
    for i in xrange(-int(npts/2),int(npts/2)):
        x = scale*i
        y = a*x + b
        z = 0.0
        lst.append((x,y,z))
    return lst


def points_from_coeffs3(coeffs,npts,scale):
    a = coeffs[0]
    b = coeffs[1]
    c = coeffs[2]

    lst = []
    for i in xrange(npts):
        x = scale*i
        y = -(c+a*x)/b
        z = 0.0
        lst.append((x,y,z))
    return lst
