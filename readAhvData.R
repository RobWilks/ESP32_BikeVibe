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

remove(list = ls())

ref_date = "1970/1/1" #for unix timestamp

setwd("C:/Users/rjwil/Documents/Computer/Arduino/Vibration/data/")

subdirName = "190427"
FileNames = Sys.glob(file.path(subdirName, "ADXL????.DAT"))
# FileNames = list.files(
#   path = subdirName,
#   pattern = "\\.dat$",
#   ignore.case = TRUE,
#   full.names = TRUE
# )

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

firstFile = TRUE

scaleUp = 16 # added by NRF51822 to allow use of integers
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

##################################### sumSquares function #####################################
sumSquares = function(x)
{
  return(sum(x * x))
}
##################################### rms function #####################################
rms = function(x)
{
  return(sqrt(mean(x*x)))
}

##################################### mag function #####################################
mag = function(x, y, z)
{
  return(sqrt(x*x + y*y + z*z))
}

##################################### read in successive accelerometer data files #####################################

xMax = 4096
sizeOfFile = xMax * 3


# for (fileNo in 1:2)
# for (fileNo in 1:length(FileNames))
for (fileNo in 1:50)
{
  cat(sprintf("Processing file %s\n", FileNames[fileNo]))
  newData = file(FileNames[fileNo], "rb")
  
  allData = readBin(newData,
                    integer(),
                    n = sizeOfFile, 
                    size = 2,
                    endian = "little")
  j = seq(1, sizeOfFile, 3)
  
  xdata = allData[j] / scaleUp
  ydata = allData[j + 1] / scaleUp
  zdata = allData[j + 2] / scaleUp
  index = rep(fileNo, xMax)
  close(newData)
  
  if (firstFile)
  {
    a8Data = data.frame(index,
                        xdata,
                        ydata,
                        zdata)
    firstFile = FALSE
    
  }
  else
  {
    df = data.frame(index,
                    xdata,
                    ydata,
                    zdata)
    a8Data = rbind(a8Data,
                   df)
  }
  
  if (fileNo < 10)
  {
  yMin = -20
  yMax = +20
  yInt = 5
  xInt = 100
  xMin_ = 1
  xMax_ = 4096
  
  windows(18, 6)
  
  plot(
    xMin_:xMax_,
    xdata[xMin_:xMax_],
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
    xMin_:xMax_,
    ydata[xMin_:xMax_],
    type = "l",
    col = palette[2],
    pch = 2,
    cex = .05,
    lwd = lineWidth
  )
  
  lines(
    xMin_:xMax_,
    zdata[xMin_:xMax_],
    type = "l",
    col = palette[3],
    pch = 2,
    cex = .05,
    lwd = lineWidth
  )
  }
  indexAsFactor = factor(a8Data$index)
}
# calculate a8 for each file, x, y and z successively
magnitude = mag(a8Data$xdata, a8Data$ydata, a8Data$zdata)
a8Data = cbind(a8Data, magnitude)
a8total = (tapply(a8Data$magnitude, indexAsFactor, sumSquares)  * (1 / 3600 / 8 / 900))^.5
df = data.frame(fileNo = levels(indexAsFactor), a8total)
print(sumSquares(c(1,2,3)))
