###############################  Infinite Impulse Response Filtering  ###############################

# From Industrial Health 2007, 45, 512–519
# Design of Digital Filters for Frequency Weightings
# Required for Risk Assessments of Workers
# Exposed to Vibration
# Andrew N. RIMELL and Neil J. MANSFIELD*

#sampling frequency
fs = 900 #Hz
# fs = 1500 #Hz


#highpass filter
f1 = 6.31 #Hz
q1 = 2^-0.5
w1 = 2 * pi * f1 / fs
w1_ = 2 * tan(w1 / 2)
Hh = c(
  4 * q1 + 2 * w1_ + w1_^2,
  2 * w1_^2 - 8 * q1,
  4 * q1 - 2 * w1_ + w1_^2,
  4 * q1,
  -8 * q1,
  4 * q1
)  

#lowpass filter
f2 = 1258.9 #Hz
q2 = 2^-0.5
w2 = 2 * pi * f2 / fs
w2_ = 2 * tan(w2 / 2)
Hl = c(
  4 * q2 + 2 * w2_ + w2_ ^ 2 * q2,
  2 * w2_ ^ 2 * q2 - 8 * q2,
  4 * q2 - 2 * w2_ + w2_ ^ 2 * q2,
  w2_ ^ 2 * q2,
  2 * w2_ ^ 2 * q2,
  w2_ ^ 2 * q2
)


#HAV weighting filter Hw
f7 = 15.915
q7 = 0.64
f8 = 15.915
q8 = 0.64

w7 = 2 * pi * f7 / fs
w7_ = 2 * tan(w7 / 2)
w8 = 2 * pi * f8 / fs
w8_ = 2 * tan(w8 / 2)

Hw = c(
  w7_ * q7 * w8_ ^ 2 + 2 * w7_ * w8_ + 4 * w7_ * q7,
  2 * w7_ * q7 * w8_ ^ 2 - 8 * w7_ * q7,
  w7_ * q7 * w8_ ^ 2 - 2 * w7_ * w8_ + 4 * w7_ * q7,
  w7_ * q7 * w8_ ^ 2 + 2 * q8 * w8_ ^ 2,
  2 * w7_ * q7 * w8_ ^ 2,
  w7_ * q7 * w8_ ^ 2 - 2 * q8 * w8_ ^ 2
)

Hw_factor = 277 / Hw[1]
Hl_factor = 10222 / Hl[1]
Hh_factor = 2909 / Hh[1]

Hw_integer = as.integer(round(Hw * Hw_factor))
Hl_integer = as.integer(round(Hl * Hl_factor))
Hh_integer = as.integer(round(Hh * Hh_factor))

# Hw_integer = as.integer(Hw * 64)
# Hl_integer = as.integer(Hl * 64)
# Hh_integer = as.integer(Hh * 64)

# Hw_integer =   c(18, -33, 15, 1, 0, -1)
# Hl_integer = c(20, 1, 4, 6, 13, 6)
# Hh_integer = c(12, -23, 11, 11, -23, 11)

# [Hw,Hl,Hh][a0,a1,a2,b0,b1,b2]


df = data.frame(Hw_integer, Hl_integer, Hh_integer)
df2 = data.frame(Hw, Hl, Hh)
df3 = data.frame(t(df[,]))
df4 = data.frame(t(df2[,]))

cat(sprintf("%i,",Hw_integer))
cat("\n")
cat(sprintf("%i,",Hl_integer))
cat("\n")
cat(sprintf("%i,",Hh_integer))

###############################  infInputResponse function  ###############################

infInputResponse = function(x, y, ab)
{
  # x is a vector of input values.  The most recent input is the last element of the vector
  # y is a vector of output values.  The function adds a new element to the end of y
  # the IIR filter is called recursively
  # coefficients ab determine different types of filter
  n = length(x)
  yn = (sum(ab[4:6] * x[n:(n-2)]) - sum(ab[2:3] * y[(n-1):(n-2)])) / ab[1]
  return(c(y,yn))
}

###############################  infInputResponse_integer function  ###############################

infInputResponse_integer = function(x, y, ab)
{
  # x is a vector of input values.  The most recent input is the last element of the vector
  # y is a vector of output values.  The function adds a new element to the end of y
  # the IIR filter is called recursively
  # coefficients ab determine different types of filter
  n = length(x)
  yn = (sum(ab[4:6] * x[n:(n-2)]) - sum(ab[2:3] * y[(n-1):(n-2)])) / ab[1]
  return(c(y,as.integer(yn)))
}

###############################  calcDigFilter function  ###############################


calcDigFilter = function(fs, f1, f2, useInteger)
{
  #sampling frequency
  # fs = 1000 #Hz
  
  
  #highpass filter
  q1 = 2^-0.5
  w1 = 2 * pi * f1 / fs
  w1_ = 2 * tan(w1 / 2)
  Hh = c(
    4 * q1 + 2 * w1_ + w1_^2,
    2 * w1_^2 - 8 * q1,
    4 * q1 - 2 * w1_ + w1_^2,
    4 * q1,
    -8 * q1,
    4 * q1
  )  
  
  #lowpass filter
  q2 = 2^-0.5
  w2 = 2 * pi * f2 / fs
  w2_ = 2 * tan(w2 / 2)
  Hl = c(
    4 * q2 + 2 * w2_ + w2_ ^ 2 * q2,
    2 * w2_ ^ 2 * q2 - 8 * q2,
    4 * q2 - 2 * w2_ + w2_ ^ 2 * q2,
    w2_ ^ 2 * q2,
    2 * w2_ ^ 2 * q2,
    w2_ ^ 2 * q2
  )
  
  
  #HAV weighting filter Hw
  f7 = 15.915
  q7 = 0.64
  f8 = 15.915
  q8 = 0.64
  
  w7 = 2 * pi * f7 / fs
  w7_ = 2 * tan(w7 / 2)
  w8 = 2 * pi * f8 / fs
  w8_ = 2 * tan(w8 / 2)
  
  Hw = c(
    w7_ * q7 * w8_ ^ 2 + 2 * w7_ * w8_ + 4 * w7_ * q7,
    2 * w7_ * q7 * w8_ ^ 2 - 8 * w7_ * q7,
    w7_ * q7 * w8_ ^ 2 - 2 * w7_ * w8_ + 4 * w7_ * q7,
    w7_ * q7 * w8_ ^ 2 + 2 * q8 * w8_ ^ 2,
    2 * w7_ * q7 * w8_ ^ 2,
    w7_ * q7 * w8_ ^ 2 - 2 * q8 * w8_ ^ 2
  )
  if(useInteger)
  {
  Hw_integer = as.integer(Hw * 1000)
  Hl_integer = as.integer(Hl * 1000)
  Hh_integer = as.integer(Hh * 1000)
  df = data.frame(Hw_integer, Hl_integer, Hh_integer)
  }
  else
  {
    df = data.frame(Hw, Hl, Hh)
    
  }
  return(df)
  
}
