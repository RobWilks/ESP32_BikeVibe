###############################  read soil sensor, reference and status data  ###############################
#
# read all files into one large data frame
# install.packages("stringr", repos='http://cran.us.r-project.org')
# previous analysis method read files three times:  for TDC data, temp data, vBatt data and timestamp
# here all are read in one pass

library(stringr)
library(magrittr)
library(pracma)
library(TTR)

# remove(list = ls())

ref_date = "1970/1/1" #for unix timestamp

setwd("C:/Users/rjwil/Documents/Computer/Arduino/Vibration/data/")

subdirName = "190411"
# FileNames = Sys.glob(file.path(subdirName, "SD_??_??.CSV"))
FileNames = list.files(
  path = subdirName,
  pattern = "\\.dat$",
  ignore.case = TRUE,
  full.names = TRUE
)

# prepare regular expression
regexp <- "\\d+\\.*\\d*"

# row limit on file read
rowLimit = 500000 # not sure whether needed


##################################### initialise constants #####################################
testSpectrum = F # when TRUE used to calibrate power spectrum
MovingAverage = F
smooth = F
plotFFT = F
plotMag = F # when FALSE plot x, y and z
plotXYZ = F
if ((plotFFT) && (plotMag)) {
  plotXYZ = F
}
firstTime = T

Colour = 1
xMax = 2048
nOmit = 128
sizeOfFile = xMax * 3 * 2
firstFile = 1
lastFile = 302
timeInterval = 2e-3 #time between accelerometer measurements in s
repeatTime = 12 #time between files in s

indices = 1:(xMax / 2)
freqs = (indices - 1) / timeInterval / xMax

xs = c(38.78132, 103.4153, 19.31993, 105.2384,-28.44483, 103.8676)
odd = c(1, 3, 5)
even = c(2, 4, 6)
scaleUp = 16 # for integer calculation
# scaleUp = mean(xs[even]) / 16 # for integer calculation
xs_integer = c(rep(0, 6))
xs_integer[odd] = as.integer(xs[odd] * 16 + (.5 * sign(xs[odd])))
xs_integer[even] = as.integer(xs[even] * 16 / scaleUp + (.5 * sign(xs[even])))

xs = xs / 16
sol = data.frame(xs)
sol_integer = data.frame(xs_integer)

palette = c(
  "blue",
  "green",
  "red",
  "violet",
  "cyan",
  "pink",
  "springgreen",
  "darkviolet",
  "orange",
  "brown",
  "black",
  "darkolivegreen4",
  "darkgray",
  "gold",
  "plum3",
  "thistle4",
  "pink",
  "azure"
)

lineWidth = .3

##################################### weighting factors and third octave values #####################################
# from C:\Users\rjwil\Documents\Computer\Arduino\Vibration\code\weighting factor one third harmonic.xlsx
wf = c(
  0,
  0.36886478493075,
  0.447212241465328,
  0.533011184230749,
  0.621677016072035,
  0.707098217636303,
  0.78325487676388,
  0.846080761159794,
  0.894383864843277,
  0.92941131854351,
  0.762950104406269,
  0.611071376485367,
  0.490287520950875,
  0.394936990958503,
  0.297452897100191,
  0.248473086297078,
  0.198994508037629,
  0.159195606430103,
  0.124236543148539,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0
)


fc = c(
  2.46078330057592,
  3.10039267962539,
  3.90625,
  4.92156660115185,
  6.20078535925078,
  7.8125,
  9.8431332023037,
  12.4015707185016,
  15.625,
  19.6862664046074,
  24.8031414370031,
  31.25,
  39.3725328092148,
  49.6062828740062,
  62.5,
  78.7450656184296,
  99.2125657480125,
  125,
  157.490131236859,
  198.425131496025,
  250,
  314.980262473718,
  396.85026299205,
  500,
  629.960524947437,
  793.7005259841,
  1000,
  1259.92104989487,
  1587.4010519682,
  2000,
  2519.84209978975,
  3174.8021039364,
  4000,
  5039.68419957949,
  6349.6042078728,
  8000
)
df = cbind(fc = c(0, fc), wf)
#freqs is defined on the half space; need wf to be defined on the full space
#find the indices of successive third octaves
#use these to calculate weighting factors for each of the measured frequencies
thirdOctaves <-
  as.integer(cut(
    freqs,
    c(0, fc / (2 ^ (1 / 6))),
    labels = 1:length(fc),
    include.lowest = TRUE
  ))
reverseThirdOctaves = thirdOctaves[length(thirdOctaves):1]
wffw = c(wf[thirdOctaves], wf[reverseThirdOctaves])
df = cbind(freqs, thirdOctaves)


##################################### grid function #####################################
grid = function(xMin, xMax, xInt, yMin, yMax, yInt)
{
  yVals <- seq(yMin, yMax, by = yInt)
  abline(h = yVals,
         v = NULL,
         col = "gray",
         lty = 3)
  
  xVals <- seq(1, sizeOfFile, by = xInt)
  abline(v = xVals,
         h = NULL,
         col = "gray",
         lty = 3)
}


##################################### read in successive accelerometer data files #####################################
# for (fileNo in 3:3)
# for (fileNo in (length(FileNames) - 5):length(FileNames))
# for (fileNo in 1:length(FileNames))
for (fileNo in 100:100)
  # for (fileNo in 1:10 * 30)
  # for (fileNo in 1:length(FileNames))
  # for (fileNo in firstFile:lastFile)
{
  cat(sprintf("Processing file %s\n", FileNames[fileNo]))
  newData = file(FileNames[fileNo], "rb")
  
  allData = readBin(newData,
                    integer(),
                    n = sizeOfFile,
                    size = 2,
                    endian = "little")
  
  j = seq(1, sizeOfFile, 3)
  
  xdata = allData[j]
  ydata = allData[j + 1]
  zdata = allData[j + 2]
  
  close(newData)
  
  evens = 1:xMax * 2
  
  adxlData = data.frame(xdata = xdata[evens],
                        ydata = ydata[evens],
                        zdata = zdata[evens])
  adxlData = na.omit(adxlData)
  
  ##################################### calculate acceleration from calibration constants #####################################
  if (testSpectrum)
  {
    calx = 10 * sin (1:xMax * 2 * pi * fileNo * timeInterval)
    caly = calx
    calz = calx
    calx_integer = as.integer(16 * calx)
    caly_integer = calx_integer
    calz_integer = calz_integer
  }
  else
  {
    calx = (adxlData$xdata - sol$xs[1]) / sol$xs[2]
    caly = (adxlData$ydata - sol$xs[3]) / sol$xs[4]
    calz = (adxlData$zdata - sol$xs[5]) / sol$xs[6]
    calx_integer = (adxlData$xdata * 256 - sol_integer$xs[1]) / sol_integer$xs[2]
    caly_integer = (adxlData$ydata * 256 - sol_integer$xs[3]) / sol_integer$xs[4]
    calz_integer = (adxlData$zdata * 256 - sol_integer$xs[5]) / sol_integer$xs[6]
    calx_integer = as.integer(calx_integer + .5 * sign(calx_integer))
    caly_integer = as.integer(caly_integer + .5 * sign(caly_integer))
    calz_integer = as.integer(calz_integer + .5 * sign(calz_integer))
  }
  # calx_integer = adxlData$xdata
  # caly_integer = adxlData$ydata
  # calz_integer = adxlData$zdata
  
  
  mag = (calx ^ 2 + caly ^ 2 + calz ^ 2) ^ 0.5
  err =   calx ^ 2 + caly ^ 2 + calz ^ 2 - (9.81 ^ 2)
  
  adxlCal = data.frame(calx, caly, calz, mag, err)
  
  ##################################### calculate moving average #####################################
  if (MovingAverage)
  {
    smoothWindow = 21
    smoothx = SMA(calx[1:xMax], smoothWindow)
    smoothy = SMA(caly[1:xMax], smoothWindow)
    smoothz = SMA(calz[1:xMax], smoothWindow)
  }
  ##################################### calculate power spectrum #####################################
  
  
  
  fftx = fft(calx[1:xMax], inverse = FALSE)
  ffty = fft(caly[1:xMax], inverse = FALSE)
  fftz = fft(calz[1:xMax], inverse = FALSE)
  fftmag = fft(mag[1:xMax], inverse = FALSE)
  ##################################### smooth power spectrum #####################################
  if (smooth)
  {
    smoothWindow = 21
    smoothfftx = SMA(Mod(fftx[1:xMax]), smoothWindow)
    smoothffty = SMA(Mod(ffty[1:xMax]), smoothWindow)
    smoothfftz = SMA(Mod(fftz[1:xMax]), smoothWindow)
    smoothfftmag = SMA(Mod(fftmag[1:xMax]), smoothWindow)
  }
  ##################################### apply weighting factors to FFT #####################################
  wfftx = fftx * wffw
  wffty = ffty * wffw
  wfftz = fftz * wffw
  
  ##################################### calculate corrected acceleration #####################################
  invfftx = fft(wfftx, inverse = TRUE) / xMax
  invffty = fft(wffty, inverse = TRUE) / xMax
  invfftz = fft(wfftz, inverse = TRUE) / xMax
  rmsx = (mean(Real(invfftx) ^ 2)) ^ .5
  rmsy = (mean(Real(invffty) ^ 2)) ^ .5
  rmsz = (mean(Real(invfftz) ^ 2)) ^ .5
  ahv = (rmsx ^ 2 + rmsy ^ 2 + rmsz ^ 2) ^ .5
  if (firstTime)
  {
    results = data.frame(fileNo, fileName = FileNames[fileNo], rmsx, rmsy, rmsz, ahv)
  }
  else
  {
    df = data.frame(fileNo, fileName = FileNames[fileNo], rmsx, rmsy, rmsz, ahv)
    results = rbind(results, df)
  }
  
  ##################################### apply digital filter #####################################
  # call digitalFilterForWeighting first!
  # initialise filters
  aa = calx[1:2]
  bb = calx[1:2]
  cc = calx[1:2]
  dd = calx[1:2]
  for (t in 3:xMax)
  {
    aa = c(aa, calx[t])
    bb = infInputResponse(aa, bb, Hw)
    cc = infInputResponse(bb, cc, Hl)
    dd = infInputResponse(cc, dd, Hh)
    # bb = infInputResponse(aa, bb, Hw)
    # cc = infInputResponse(bb, cc, Hl)
    # dd = infInputResponse(cc, dd, Hh)
  }
  filterx = dd
  
  # initialise filters
  aa = caly[1:2]
  bb = caly[1:2]
  cc = caly[1:2]
  dd = caly[1:2]
  for (t in 3:xMax)
  {
    aa = c(aa, caly[t])
    bb = infInputResponse(aa, bb, Hw)
    cc = infInputResponse(bb, cc, Hl)
    dd = infInputResponse(cc, dd, Hh)
    # bb = infInputResponse(aa, bb, Hw)
    # cc = infInputResponse(bb, cc, Hl)
    # dd = infInputResponse(cc, dd, Hh)
  }
  filtery = dd
  
  # initialise filters
  aa = calz[1:2]
  bb = calz[1:2]
  cc = calz[1:2]
  dd = calz[1:2]
  for (t in 3:xMax)
  {
    aa = c(aa, calz[t])
    bb = infInputResponse(aa, bb, Hw)
    cc = infInputResponse(bb, cc, Hl)
    dd = infInputResponse(cc, dd, Hh)
    # bb = infInputResponse(aa, bb, Hw)
    # cc = infInputResponse(bb, cc, Hl)
    # dd = infInputResponse(cc, dd, Hh)
  }
  filterz = dd
  filterOut = data.frame(filterx, filtery, filterz)
  
  ##################################### apply integer digital filter #####################################
  # call digitalFilterForWeighting first!
  # initialise filters
  aa = as.integer(calx_integer[1:2])
  bb = aa
  cc = aa
  dd = aa
  
  for (t in 3:xMax)
  {
    aa = c(aa, as.integer(calx_integer[t]))
    bb = infInputResponse_integer(aa, bb, Hw_integer)
    cc = infInputResponse_integer(bb, cc, Hl_integer)
    dd = infInputResponse_integer(cc, dd, Hh_integer)
    # if (is.na(dd[length(dd)]))
    # {cat("Error")
    #   browser()}
  }
  filterxInteger = dd #/ scaleUp
  
  # initialise filters
  aa = as.integer(caly_integer[1:2])
  bb = aa
  cc = aa
  dd = aa
  for (t in 3:xMax)
  {
    aa = c(aa, as.integer(caly_integer[t]))
    bb = infInputResponse_integer(aa, bb, Hw_integer)
    cc = infInputResponse_integer(bb, cc, Hl_integer)
    dd = infInputResponse_integer(cc, dd, Hh_integer)
  }
  filteryInteger = dd #/ scaleUp
  
  # initialise filters
  aa = as.integer(calz_integer[1:2])
  bb = aa
  cc = aa
  dd = aa
  for (t in 3:xMax)
  {
    aa = c(aa, as.integer(calz_integer[t]))
    bb = infInputResponse_integer(aa, bb, Hw_integer)
    cc = infInputResponse_integer(bb, cc, Hl_integer)
    dd = infInputResponse_integer(cc, dd, Hh_integer)
  }
  filterzInteger = dd #/ scaleUp
  filterOut = cbind(filterOut, filterxInteger, filteryInteger, filterzInteger)
  ##################################### calc ahv for digital filter #####################################
  
  rmsx = (mean(filterx[(nOmit + 1):xMax] ^ 2)) ^ .5
  rmsy = (mean(filtery[(nOmit + 1):xMax] ^ 2)) ^ .5
  rmsz = (mean(filterz[(nOmit + 1):xMax] ^ 2)) ^ .5
  ahv = (rmsx ^ 2 + rmsy ^ 2 + rmsz ^ 2) ^ .5
  if (firstTime)
  {
    resultsDig = data.frame(fileNo, fileName = FileNames[fileNo], rmsx, rmsy, rmsz, ahv)
  }
  else
  {
    df = data.frame(fileNo, fileName = FileNames[fileNo], rmsx, rmsy, rmsz, ahv)
    resultsDig = rbind(resultsDig, df)
  }
  
  ##################################### calc ahv for digital integer filter #####################################
  
  rmsx = (mean(filterxInteger[(nOmit + 1):xMax] ^ 2)) ^ .5
  rmsy = (mean(filteryInteger[(nOmit + 1):xMax] ^ 2)) ^ .5
  rmsz = (mean(filterzInteger[(nOmit + 1):xMax] ^ 2)) ^ .5
  ahv = (rmsx ^ 2 + rmsy ^ 2 + rmsz ^ 2) ^ .5
  if (firstTime)
  {
    resultsDigInteger = data.frame(fileNo, fileName = FileNames[fileNo], rmsx, rmsy, rmsz, ahv)
  }
  else
  {
    df = data.frame(fileNo, fileName = FileNames[fileNo], rmsx, rmsy, rmsz, ahv)
    resultsDigInteger = rbind(resultsDigInteger, df)
  }
  
  ##################################### plot calibrated & weighted accelerometer data #####################################
  if (plotXYZ)
  {
    yMin = -20
    yMax = +20
    yInt = 5
    xInt = 100
    xMax_ = 256
    
    windows(18, 6)
    
    plot(
      1:xMax_,
      calx[1:xMax_],
      type = "l",
      # xlim = c(xmin, xmax),
      ylim = c(yMin, yMax),
      col = palette[1],
      pch = 2,
      cex = .05,
      lwd = lineWidth,
      main = paste("X Axis Acceleration for", fileNo),
      sub = "Calibrated",
      xlab = "Index",
      ylab = "Acceleration (ms^-2)"
    )
    
    grid(1, xMax_, xInt, yMin, yMax, yInt)
    
    lines(
      1:xMax_,
      Real(invfftx[1:xMax_]),
      type = "l",
      col = palette[2],
      pch = 2,
      cex = .05,
      lwd = lineWidth
    )
    
    lines(
      1:xMax_,
      filterx[1:xMax_],
      type = "l",
      col = palette[3],
      pch = 2,
      cex = .05,
      lwd = lineWidth
    )
    
    if (MovingAverage)
    {
      lines(
        1:xMax_,
        smoothx,
        type = "l",
        col = palette[3],
        pch = 2,
        cex = .3,
        lwd = lineWidth
      )
    }
    if (!testSpectrum) # don't plot y and z for test spectrum
    {
      windows(18, 6)
      
      plot(
        1:xMax_,
        caly[1:xMax_],
        type = "l",
        # xlim = c(xmin, xmax),
        ylim = c(yMin, yMax),
        col = palette[4],
        pch = 2,
        cex = .05,
        lwd = lineWidth,
        main = paste("Y Axis Acceleration for", fileNo),
        sub = "Calibrated",
        xlab = "Index",
        ylab = "Acceleration (ms^-2)"
      )
      
      grid(1, xMax_, xInt, yMin, yMax, yInt)
      
      lines(
        1:xMax_,
        Real(invffty[1:xMax_]),
        type = "l",
        col = palette[5],
        pch = 2,
        cex = .05,
        lwd = lineWidth
      )
      
      lines(
        1:xMax_,
        filtery[1:xMax_],
        type = "l",
        col = palette[3],
        pch = 2,
        cex = .05,
        lwd = lineWidth
      )
      
      if (MovingAverage)
      {
        lines(
          1:xMax_,
          smoothy,
          type = "l",
          col = palette[6],
          pch = 2,
          cex = .3,
          lwd = lineWidth
        )
      }
      windows(18, 6)
      
      plot(
        1:xMax_,
        calx[1:xMax_],
        type = "l",
        # xlim = c(xmin, xmax),
        ylim = c(yMin, yMax),
        col = palette[7],
        pch = 2,
        cex = .05,
        lwd = lineWidth,
        main = paste("Z Axis Acceleration for", fileNo),
        sub = "Calibrated",
        xlab = "Index",
        ylab = "Acceleration (ms^-2)"
      )
      
      grid(1, xMax_, xInt, yMin, yMax, yInt)
      
      lines(
        1:xMax_,
        Real(invfftz[1:xMax_]),
        type = "l",
        col = palette[8],
        pch = 2,
        cex = .05,
        lwd = lineWidth
      )
      
      lines(
        1:xMax_,
        filterz[1:xMax_],
        type = "l",
        col = palette[3],
        pch = 2,
        cex = .05,
        lwd = lineWidth
      )
      
      if (MovingAverage)
      {
        lines(
          1:xMax_,
          smoothz,
          type = "l",
          col = palette[9],
          pch = 2,
          cex = .3,
          lwd = lineWidth
        )
      }
    }
  }
  ##################################### plot fft #####################################
  if (plotFFT)
  {
    yMin = 0
    yMax = 10000
    yInt = 1000
    xInt = 10
    if (plotMag)
    {
      if (Colour == 1)
      {
        windows(18, 6)
        plot(
          freqs,
          smoothfftmag[indices],
          type = "l",
          # xlim = c(xmin, xmax),
          ylim = c(yMin, yMax),
          col = palette[Colour],
          pch = 2,
          cex = .05,
          lwd = lineWidth,
          main = paste("Acceleration Magnitude for", fileNo),
          sub = "Calibrated",
          xlab = "Index",
          ylab = "Acceleration (ms^-2)"
        )
        grid(0, max(freqs), 100, yMin, yMax, yInt)
        
      }
      else
      {
        lines(
          freqs,
          smoothfftmag[indices],
          type = "l",
          col = palette[Colour],
          pch = 2,
          cex = .05,
          lwd = lineWidth
        )
        
      }
    }
    else
    {
      windows(18, 6)
      plot(
        freqs,
        smoothfftx[indices],
        type = "l",
        # xlim = c(xmin, xmax),
        ylim = c(yMin, yMax),
        col = palette[1],
        pch = 2,
        cex = .05,
        lwd = lineWidth,
        main = paste("X/Y/Z Acceleration for", fileNo),
        sub = "Calibrated",
        xlab = "Index",
        ylab = "Acceleration (ms^-2)"
      )
      grid(0, max(freqs), 100, yMin, yMax, yInt)
      
      
      
      lines(
        freqs,
        smoothffty[indices],
        type = "l",
        col = palette[2],
        pch = 2,
        cex = .05,
        lwd = lineWidth
      )
      
      
      lines(
        freqs,
        smoothfftz[indices],
        type = "l",
        col = palette[3],
        pch = 2,
        cex = .05,
        lwd = lineWidth
      )
    }
  }
  Colour = Colour + 1
  firstTime = FALSE
} # end for loop


##################################### print a8 values #####################################

# see https://www.researchgate.net/publication/316926852_Cyclist_exposure_to_hand-arm_vibration_and_pavement_surface_improvement_in_the_City_of_Edinburgh

# Directive 2002/44/EC of European Parliament Council (2002) provides details of the minimum health and safety requirements
# regarding exposure of workers to the risks arising from mechanical vibration.
# The Directive defines exposure limit values for hand-arm vibration based upon a standardized eight hour reference period (simulating a working day).
# The Directive defines the thresholds of hand-arm vibration exposure from which the employer has to control
# (Exposure Action Value, EAV = 2.5 ms^-2 rms) and threshold exposure limits to which workers must not be subjected
# (Exposure Limit Value, ELV = 5.0 ms^-2 rms)
a8 = (repeatTime / 8 / 3600 * sum(results$ahv ^ 2)) ^ .5
str1 = sprintf("Total a8 exposure from analysis %5.2f ms-2 rms\n", a8)
str2 = sprintf("Total time %i s\n", repeatTime * length(results$ahv))
cat(str1)
cat("Exposure Action Value, EAV = 2.5 ms^-2 rms\n")
cat("Exposure Limit Value, ELV = 5.0 ms^-2 rms\n")
cat(str2)

stationary = which(results$ahv < 0.1)
cat("Files when stationary\n")
cat(" ")
cat(sprintf("%10s %5.2f\n", FileNames[stationary] , results$ahv[stationary]))

a8 = (repeatTime / 8 / 3600 * sum(resultsDig$ahv ^ 2)) ^ .5
str1 = sprintf("Total a8 exposure from analysis %5.2f ms-2 rms\n", a8)
str2 = sprintf("Total time %i s\n", repeatTime * length(resultsDig$ahv))
cat(str1)
cat("Exposure Action Value, EAV = 2.5 ms^-2 rms\n")
cat("Exposure Limit Value, ELV = 5.0 ms^-2 rms\n")
cat(str2)

stationary = which(resultsDig$ahv < 0.1)
cat("Files when stationary\n")
cat(" ")
cat(sprintf("%10s %5.2f\n", FileNames[stationary] , resultsDig$ahv[stationary]))








##################################### plot a8 values #####################################


yMin = 0
yMax = 20
yInt = 1
# yMin = 0
# yMax = 2
# yInt = .1
windows(18, 6)
plot(
  # 1:length(results$ahv),
  1:length(results$ahv) * repeatTime,
  results$ahv,
  type = "l",
  # xlim = c(xmin, xmax),
  ylim = c(yMin, yMax),
  col = palette[3],
  pch = 2,
  cex = .7,
  lwd = lineWidth,
  main = paste(str1, str2),
  sub = paste("Measured", subdirName),
  xlab = "Index",
  ylab = "Ahv (ms^-2)"
)

lines(
  # 1:length(resultsDig$ahv),
  1:length(resultsDig$ahv) * repeatTime,
  resultsDig$ahv,
  type = "l",
  col = palette[1],
  pch = 2,
  cex = .7,
  lwd = lineWidth,
  main = paste(str1, str2)
)

# lines(
#   c(0,fc),
#   # 1:length(resultsDig$ahv) * repeatTime,
#   wf * (3/2)^(1/2),
#   type = "b",
#   col = palette[3],
#   pch = 2,
#   cex = .7,
#   lwd = lineWidth,
#   main = paste(str1, str2)
# )

grid(0, length(results$ahv), 100, yMin, yMax, yInt)

##################################### plot a8 values integer #####################################


yMin = 0
yMax = 20
yInt = 1
# yMin = 0
# yMax = 2
# yInt = .1
windows(18, 6)
plot(
  # 1:length(results$ahv),
  1:length(resultsDig$ahv) * repeatTime,
  resultsDig$ahv,
  type = "l",
  # xlim = c(xmin, xmax),
  ylim = c(yMin, yMax),
  col = palette[1],
  pch = 2,
  cex = .7,
  lwd = lineWidth,
  main = paste(str1, str2),
  sub = paste("Measured", subdirName),
  xlab = "Index",
  ylab = "Ahv (ms^-2)"
)

lines(
  # 1:length(resultsDig$ahv),
  1:length(resultsDigInteger$ahv) * repeatTime,
  resultsDigInteger$ahv / 16,
  type = "l",
  col = palette[2],
  pch = 2,
  cex = .7,
  lwd = lineWidth,
  main = paste(str1, str2)
)

# lines(
#   c(0,fc),
#   # 1:length(resultsDig$ahv) * repeatTime,
#   wf * (3/2)^(1/2),
#   type = "b",
#   col = palette[3],
#   pch = 2,
#   cex = .7,
#   lwd = lineWidth,
#   main = paste(str1, str2)
# )

grid(0, length(results$ahv), 100, yMin, yMax, yInt)
##################################### plot time markers #####################################

mins = c(5, 9, 15, 18, 22, 26)
secs = c(33, 3, 24, 39, 12, 3)
markerTimes = as.integer((mins * 60 + secs) / repeatTime * 1610 / 1560)
mins2 = c(5, 8, 20, 27, 40, 48)
secs2 = c(59, 38, 12, 25, 41, 56)
markerTimes2 = as.integer((mins2 * 60 + secs2) / repeatTime) + max(markerTimes)
markers = c(markerTimes, markerTimes2)


lines(
  markers * repeatTime,
  results$ahv[markers],
  type = "p",
  pch = 1,
  cex = 1,
  col = 2
)



##################################### end #####################################
df4 <- resultsDig
df5 = cbind(df4 , ahv2 = resultsDig$ahv)
windows(18, 6)
yMin = 0
yMax = 40
yInt = 5
plot(
  1:length(df5$fileNo) * 10,
  df5$ahv / df5$ahv2 * 100 - 100,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(yMin, yMax),
  col = palette[1],
  pch = 2,
  cex = .7,
  lwd = lineWidth,
  main = "Error caused by IIR filter settling time",
  xlab = "Frequence (Hz)",
  ylab = "%"
)

grid(1, length(results$ahv), 10, yMin, yMax, yInt)

windows(18, 6)
yMin = 0
yMax = 1
yInt = .1
plot(
  1:length(df5$fileNo) * 10,
  df5$ahv - df5$ahv2,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(yMin, yMax),
  col = palette[1],
  pch = 2,
  cex = .7,
  lwd = lineWidth,
  main = "Error caused by IIR filter settling time",
  xlab = "Frequence (Hz)",
  ylab = "delta Ahv RMS ms-2"
)

grid(1, length(results$ahv), 10, yMin, yMax, yInt)




hfResults <- results
hfResultsDig <- resultsDig

df6 <- cbind(results, hfResults$ahv)
df6 <- cbind(df6, results$ahv)

