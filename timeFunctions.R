###############################  dayofyear   ###############################

dayofyear = function(timedata, ref) {
  #days since 1/1/15 0000h, i.e.unix timestamp 1420070400
  # '0' for first day of year
  # this function uses the next one
  # posix returns a list of time attributes, month, day, yday etc. Here use yday
  return(posixtime(timedata, ref)[[8]])
  
}

###############################  hourofday   ###############################

hourofday = function(timedata, ref) {
  #hours since 1/1/15 0000h, i.e.unix timestamp 1420070400
  # '0' for first day of year
  # this function uses the next one
  # posix returns a list of time attributes, month, day, yday etc. Here use yday
  return(posixtime(timedata, ref)[[3]])
  
}
###############################  hourofyear   ###############################

hourofyear = function(timedata, ref) {
  #hours since 1/1/15 0000h, i.e.unix timestamp 1420070400
  # '0' for first day of year
  # posix returns a list of time attributes, month, day, yday etc. Here use yday
  return(posixtime(timedata, ref)[[3]] + 24 * posixtime(timedata, ref)[[8]])
  
}

###############################  hourofyear   ###############################

time_interval_of_year = function(timedata, ref, interval) {
  #intervals since 1/1/15 0000h, i.e.unix timestamp 1420070400
  # '0' for first interval of year
  # posix returns a list of time attributes, month, day, yday etc. Here use yday
  return((posixtime(timedata, ref)[[2]] +
            60 * (
              posixtime(timedata, ref)[[3]] + 24 * posixtime(timedata, ref)[[8]]
            )) %/% interval)
  
}

###############################  posix_unixtime   ###############################
posixtime = function(timestamp, reference_date) {
  return(as.POSIXlt(timestamp, origin = reference_date))
}

###############################  unix_timestring   ###############################

unixtime = function(charstring, reference_date) {
  # returns unixtime as seconds since reference_date
  # charstring format example: "7/13/2015 19:52:25"
  test = strptime(charstring, "%m/%d/%Y %H:%M:%S", tz = "GMT")
  return (as.integer(as.POSIXct(test, origin = reference_date, tz = "GMT")))
}

###############################  posix_timestring   ###############################

posixFromString = function(charstring, reference_date) {
  # returns posixtime from charstring
  # charstring format example: "7/13/2015 19:52:25"
  test = strptime(charstring, "%m/%d/%Y %H:%M:%S", tz = "GMT")
  return (as.POSIXlt(test, origin = reference_date, tz = "GMT"))
}

###############################  radiansFromString    ###############################

radiansFromString = function(charstring, reference_date) {
  myTime = posixFromString (charstring, reference_date)
  return ((myTime$sec + myTime$min * 60 + myTime$hour * 3600)/3600/24*2*pi)
}

reference = "1970/1/1"
testTime = "05/21/2019 01:00:00"
print(radiansFromString(testTime, reference) / pi)
testTime = "05/21/2019 00:20:00"
print(radiansFromString(testTime, reference) / pi)
testTime = "05/21/2019 04:00:00"
print(radiansFromString(testTime, reference) / pi)
testTime = "05/21/2019 18:00:00"
print(radiansFromString(testTime, reference) / pi)
testTime = "05/21/2019 16:00:00"
print(radiansFromString(testTime, reference) / pi)
print(as.POSIXlt(1558473326 , origin = "1970/1/1", tz = "GMT")[[2]])
print(as.POSIXlt(1558473326 , origin = "1970/1/1", tz = "GMT"))
print(as.POSIXlt(1558473326 , origin = "1970/1/1", tz = "GMT")[2])
this = (as.POSIXlt(1558473326 , origin = "1970/1/1", tz = "GMT"))
print(unclass(this))
###############################  filename_unixtime   ###############################

#function to create string for filename from UNIX time
# convert unix time to format yy-mm-dd
filename_unixtime = function(timestamp) {
  fname = as.character(posixtime(timestamp))
  substr(fname, 5, 5) = "_"
  substr(fname, 8, 8) = "_"
  substr(fname, 1, 10)
}

###############################  end   ###############################
