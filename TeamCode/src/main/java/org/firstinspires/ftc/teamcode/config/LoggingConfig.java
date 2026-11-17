package org.firstinspires.ftc.teamcode.config;
/** Shared CSV retention budget, independent of total Control Hub storage. */
public final class LoggingConfig {
 private LoggingConfig() { }
 public static final int MAX_RUNS=50;
 public static final long BUDGET_BYTES=100L*1024*1024;
 public static final double CLEANUP_FRACTION=0.80;
 public static final long MIN_FREE_BYTES=100L*1024*1024;
}