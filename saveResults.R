# get system time
now = Sys.time()
# convert to date & timestamp
# create file name
# saveFileName = paste("ahv",strftime(now,format = "%Y%m%d_%H%M%S"),".csv")


saveFileName = paste0("ahv ",subdirName,".csv")


write.table(results, file = saveFileName,
            row.names=FALSE, 
            na="",
            col.names = TRUE,
            sep=",")

##################################### save all results  #####################################
df = cbind(results, resultsDig$ahv, resultsDigInteger$ahv)

saveFileName = paste0("all ahv results 190430",subdirName,".csv")


write.table(df, file = saveFileName,
            row.names=FALSE, 
            na="",
            col.names = TRUE,
            sep=",")

##################################### save digFilter coeffs #####################################


saveFileName = paste0("FilterCoeffs ",subdirName,".csv")


write.table(df, file = saveFileName,
            row.names=FALSE, 
            na="",
            col.names = TRUE,
            sep=",")

##################################### save integer digFilter coeffs #####################################



saveFileName = paste0("FilterCoeffs_integer ",subdirName,".csv")


write.table(df, file = saveFileName,
            row.names=FALSE, 
            na="",
            col.names = TRUE,
            sep=",")


##################################### save comparison with frequency cut off at 125Hz #####################################




saveFileName = paste0("FreqCutoff125Hz", subdirName, ".csv")


write.table(
  df6,
  file = saveFileName,
  row.names = FALSE,
  na = "",
  col.names = TRUE,
  sep = ","
)


df6 = read.table(
  file = saveFileName,
  header = T,
  na = "",
  sep = ","
)

##################################### set-up test data #####################################

adxlData = cbind(adxlData, filterxInteger, filteryInteger, filterzInteger)

saveFileName = paste0("TestData ",subdirName,".csv")


write.table(adxlData, file = saveFileName,
            row.names=FALSE, 
            na="",
            col.names = TRUE,
            sep=",")
testx = xdata
testx[testx<0] = testx[testx<0] + 65536
testy = ydata
testy[testy<0] = testy[testy<0] + 65536
testz = zdata
testz[testz<0] = testz[testz<0] + 65536
for (i in 1:512)
{
  cat(sprintf("%i,%i,",testx[i]%%256, testx[i]%/%256))
  cat(sprintf("%i,%i,",testy[i]%%256, testy[i]%/%256))
  cat(sprintf("%i,%i,\n",testz[i]%%256, testz[i]%/%256))
}

