##################################### read in accelerometer data #####################################

sizeOfFile = 4096 * 3

newData = file(FileNames[1], "rb")

allData = readBin(newData,
                  integer(),
                  n = sizeOfFile,
                  size = 2,
                  endian = "little")

indices = seq(1, sizeOfFile, 3)

xdata = allData[indices]
ydata = allData[indices + 1]
zdata = allData[indices + 2]

close(newData)

adxlData = data.frame(xdata,
                      ydata,
                      zdata)
adxlData = na.omit(adxlData)
##################################### plot accelerometer data #####################################

xMax = length(indices)
yMin = -300
yMax = +300
yInt = 50
xInt = 100
lineWidth = .5
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

windows(18, 6)
layout(matrix(1:3, 3, 1))

plot(
  # strptime(as.POSIXlt(show_data$Time, origin = ref_date),"%d/%m %H:%M"),
  1:xMax,
  adxlData$xdata,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(yMin, yMax),
  col = palette[1],
  pch = 2,
  cex = .3,
  lwd = lineWidth,
  main = "X Axis Acceleration",
  sub = "Not calibrated",
  xlab = "Time",
  ylab = "Acceleration (ms^-2)"
)

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

xMax = length(indices)
yMin = -300
yMax = +300
yInt = 50
xInt = 100

plot(
  # strptime(as.POSIXlt(show_data$Time, origin = ref_date),"%d/%m %H:%M"),
  1:xMax,
  adxlData$ydata,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(yMin, yMax),
  col = palette[2],
  pch = 2,
  cex = .3,
  lwd = lineWidth,
  main = "Y Axis Acceleration",
  sub = "Not calibrated",
  xlab = "Time",
  ylab = "Acceleration (ms^-2)"
)

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

xMax = length(indices)
yMin = -300
yMax = +300
yInt = 50
xInt = 100

plot(
  # strptime(as.POSIXlt(show_data$Time, origin = ref_date),"%d/%m %H:%M"),
  1:xMax,
  adxlData$zdata,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(yMin, yMax),
  col = palette[3],
  pch = 2,
  cex = .3,
  lwd = lineWidth,
  main = "Z Axis Acceleration",
  sub = "Not calibrated",
  xlab = "Time",
  ylab = "Acceleration (ms^-2)"
)

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


##################################### plot acceleration magnitude data #####################################

xMax = length(indices)
yMin = 250
yMax = 300
yInt = 5
xInt = 100
lineWidth = .5
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

windows(16, 9)

magnitude = sqrt(adxlData$xdata ^ 2 + adxlData$ydata ^ 2 + adxlData$zdata ^
                   2)


plot(
  1:xMax,
  magnitude,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(yMin, yMax),
  col = palette[1],
  pch = 2,
  cex = .3,
  lwd = lineWidth,
  main = "Magnitude of Acceleration",
  sub = "Not calibrated",
  xlab = "Time",
  ylab = "Acceleration (ms^-2)"
)


fn <- function(x) {
  ((adxlData$xdata - x[1]) / x[2]) ^ 2 +
    ((adxlData$ydata - x[3]) / x[4]) ^ 2 +
    ((adxlData$zdata - x[5]) / x[6]) ^ 2 - (9.81 ^ 2)
}
sol = gaussNewton(c(0, 256, 0, 256, 0, 256), fn)


calx = (adxlData$xdata - sol$xs[1]) / sol$xs[2]
caly = (adxlData$ydata - sol$xs[3]) / sol$xs[4]
calz = (adxlData$zdata - sol$xs[5]) / sol$xs[6]
mag = (calx ^ 2 + caly ^ 2 + calz ^ 2) ^ 0.5
err =   calx ^ 2 + caly ^ 2 + calz ^ 2 - (9.81 ^ 2)

adxlCal = data.frame(calx, caly, calz, mag, err)


cat(sol$xs / 16)
##################################### plot calibrated accelerometer data #####################################

xMax = length(calx)
yMin = -10
yMax = +10
yInt = 1
xInt = 5
lineWidth = .5
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

windows(18, 6)

plot(
  1:xMax,
  calx,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(yMin, yMax),
  col = palette[1],
  pch = 2,
  cex = .3,
  lwd = lineWidth,
  main = "X/Y/Z Axis Acceleration",
  sub = "Calibrated",
  xlab = "Index",
  ylab = "Acceleration (ms^-2)"
)

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

lines(
  1:xMax,
  caly,
  type = "p",
  col = palette[2],
  pch = 2,
  cex = .3,
  lwd = lineWidth
)


lines(
  1:xMax,
  calz,
  type = "p",
  col = palette[3],
  pch = 2,
  cex = .3,
  lwd = lineWidth
)

##################################### plot errors #####################################

xMax = length(calx)
yMin = 0
yMax = 1
yInt = 1e-1
xInt = 5
lineWidth = .5
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

windows(18, 6)

plot(
  1:xMax,
  abs(adxlCal$err) ^ 0.5,
  type = "p",
  # xlim = c(xmin, xmax),
  ylim = c(yMin, yMax),
  col = palette[1],
  pch = 2,
  cex = .3,
  lwd = lineWidth,
  main = "X/Y/Z Axis Acceleration",
  sub = "Calibrated",
  xlab = "Index",
  ylab = "Acceleration (ms^-2)"
)

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
