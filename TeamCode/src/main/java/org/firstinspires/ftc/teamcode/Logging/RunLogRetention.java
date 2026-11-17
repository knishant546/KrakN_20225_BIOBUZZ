package org.firstinspires.ftc.teamcode.Logging;
import java.io.*;
import java.util.*;
/** Background cleanup: only owned CSV files, oldest first; protect active runs. */
final class RunLogRetention {
 static final Set<String> ACTIVE=new HashSet<>();
 static synchronized boolean makeRoom(File dir,File current,long incoming,int maxRuns,long limit) throws IOException {
  File[] files=dir.listFiles(f->f.isFile() && f.getName().startsWith("BioBuzzRun-") && f.getName().endsWith(".csv"));
  if(files==null)throw new IOException("Cannot list logs");
  Arrays.sort(files,Comparator.comparingLong(File::lastModified).thenComparing(File::getName));
  long bytes=incoming;int count=files.length;
  for(File f:files)bytes+=f.length();
  for(File f:files){
   if(count<=maxRuns && bytes<limit)break;
   if(f.equals(current)||ACTIVE.contains(f.getAbsolutePath()))continue;
   long n=f.length();if(!f.delete())throw new IOException("Cannot delete old run "+f.getName());
   count--;bytes-=n;
  }
  return count<=maxRuns && bytes<limit;
 }
}