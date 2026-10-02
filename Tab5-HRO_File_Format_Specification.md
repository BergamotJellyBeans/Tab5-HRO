# Tab5-HRO Observation Data File Format

**Document version:** 0.1 Draft\
**File format version:** 1\
**Project:** Tab5-HRO

## 1. Purpose

This document defines the observation data file format used by
**Tab5-HRO**.

The format is designed for long-term HRO (meteor radio observation)
recording with the following goals:

-   one file represents one clock hour;
-   FFT spectra are stored at one-second intervals;
-   observation metadata is self-describing and human-readable;
-   FFT data is stored in compact binary form;
-   missing seconds can be identified;
-   RTC-only operation is supported when no network is available;
-   transition from RTC time to NTP-synchronized time can be handled
    without corrupting existing records;
-   SD-card writes must not block or disturb SDR sampling, DSP, or FFT
    processing.

The format takes inspiration from FITS in that a readable text header
precedes the binary observation data, but it is a Tab5-HRO-specific
format and is not a FITS file.

## 2. Recommended file name and directory layout

Recommended extension:

``` text
.hro
```

Recommended directory structure:

``` text
/tab5-hro/data/YYYYMMDD/
```

Recommended file name:

``` text
HRO_YYYYMMDD_HH0000.hro
```

Example:

``` text
/tab5-hro/data/20260920/HRO_20260920_090000.hro
```

A file corresponds to the civil-clock hour from `HH:00:00` through
`HH:59:59`.

If observation starts partway through an hour, the earlier second slots
are marked invalid. A new file is selected when the timestamp crosses
into another clock hour.

## 3. Overall file layout

Version 1 uses the following conceptual layout:

``` text
+--------------------------------------------------+
| Text Header                         8192 bytes    |
+--------------------------------------------------+
| Validity Map                         512 bytes    |
+--------------------------------------------------+
| FFT Slot 0       second 00:00       N x float32  |
| FFT Slot 1       second 00:01       N x float32  |
| ...                                              |
| FFT Slot 3599    second 59:59       N x float32  |
+--------------------------------------------------+
```

Where `N` is the FFT bin count recorded in the header.

For the current default observation range of 530--1030 Hz inclusive:

``` text
N = 501 bins
```

The binary data offset is therefore:

``` text
8192 + 512 = 8704 bytes
```

## 4. Text header

### 4.1 Size and encoding

The header has a fixed size of:

``` text
8192 bytes
```

The header is UTF-8 text using `KEY=VALUE` records separated by newline
characters.

The header terminates logically with:

``` text
END_HEADER
```

Unused bytes up to byte 8191 should be filled with ASCII space (`0x20`)
or another value explicitly defined by a future format revision.

Readers must use `HEADER_SIZE` rather than searching for binary data
immediately after `END_HEADER`.

### 4.2 Recommended Version 1 header fields

A Version 1 file should contain as much applicable observation
information as possible. Example:

``` text
HRO_FILE_VERSION=1
HEADER_SIZE=8192
VALIDITY_MAP_SIZE=512
DATA_TYPE=FLOAT32
BYTE_ORDER=LITTLE_ENDIAN

FILE_START_UTC=2026-09-20T00:00:00Z
FILE_HOUR_LOCAL=2026-09-20T09:00:00
TIME_ZONE=Asia/Tokyo
START_TIME_SOURCE=RTC
NTP_SYNCED=1
NTP_SYNC_TIME_UTC=2026-09-20T00:15:11Z
TIME_CONFLICTS=0

OBSERVER=...
LOCATION=...
LONGITUDE=133.345833
LATITUDE=35.443611

RECEIVER=RTL-SDR Blog V4
ANTENNA=...
RF_FREQUENCY_HZ=53750000
SDR_LO_FREQUENCY_HZ=53749000
SDR_SAMPLE_RATE_HZ=960000

FFT_SIZE=8192
FFT_OUTPUT_RATE_HZ=8192
FFT_RESOLUTION_HZ=1.0
FFT_CENTER_HZ=780
FFT_RANGE_HZ=250
FFT_MIN_HZ=530
FFT_MAX_HZ=1030
FFT_BIN_COUNT=501

RECORD_INTERVAL_SEC=1
RECORD_SLOTS=3600

DEVICE=M5Stack Tab5
SOFTWARE=Tab5-HRO
SOFTWARE_VERSION=1.0

END_HEADER
```

Additional fields may be added without changing the basic Version 1
binary layout, provided existing field meanings are not changed.

Readers should ignore unknown header keys.

## 5. Validity map

The validity map follows immediately after the 8192-byte header.

Reserved size:

``` text
512 bytes
```

Only 3600 bits are required for one hour:

``` text
3600 / 8 = 450 bytes
```

The remaining bytes are reserved for future use.

Each bit corresponds to one second slot.

``` text
0 = no valid FFT observation for this second
1 = valid FFT observation is present
```

The exact bit ordering within each byte must be fixed before Format
Version 1 is declared final. The recommended convention is:

``` text
slot s
byte index = s / 8
bit index  = s % 8
mask       = 1 << bit_index
```

Thus bit 0 of byte 0 represents second 0, bit 1 represents second 1, and
so on.

A reader must consult the validity map before interpreting an FFT slot
as an observation.

## 6. FFT binary data

### 6.1 Data type

Each FFT value is stored as:

``` text
IEEE-754 32-bit floating point
little-endian
```

The values represent the same FFT level values produced by Tab5-HRO for
its spectrum/waterfall processing. The precise amplitude normalization
and physical unit should be documented before Version 1 is frozen if
calibrated interoperability is required.

### 6.2 Fixed one-second slots

Each file contains exactly:

``` text
3600 slots
```

Slot number is calculated from the timestamp:

``` text
slot = minute * 60 + second
```

Examples:

``` text
09:00:00 -> slot 0
09:00:01 -> slot 1
09:15:20 -> slot 920
09:59:59 -> slot 3599
```

The timestamp itself is not stored with every FFT record in the file.

The time represented by a slot is derived from:

-   the clock hour represented by the file; and
-   the slot number.

### 6.3 Slot size

For `N` FFT bins:

``` text
slot_size = N * 4 bytes
```

For the default 501-bin configuration:

``` text
slot_size = 501 * 4
          = 2004 bytes
```

### 6.4 FFT slot offset

For slot `s`:

``` text
offset =
    HEADER_SIZE
  + VALIDITY_MAP_SIZE
  + s * FFT_BIN_COUNT * 4
```

With Version 1 constants:

``` text
offset =
    8192
  + 512
  + s * FFT_BIN_COUNT * 4
```

For 501 bins:

``` text
offset = 8704 + s * 2004
```

## 7. File size

For 501 bins:

``` text
header       =       8,192 bytes
validity map =         512 bytes
FFT data     = 3600 * 501 * 4
             =   7,214,400 bytes
--------------------------------
total        =   7,223,104 bytes
```

This is approximately 6.89 MiB per hour.

At 24 files per day, the nominal data volume is approximately 165
MiB/day, excluding filesystem allocation overhead.

## 8. Missing observations

Missing data is represented primarily by the validity map.

Examples include:

-   Tab5-HRO starts after the beginning of the hour;
-   observation temporarily stops;
-   an FFT result cannot be queued for storage;
-   the SD writer cannot save a record;
-   system time jumps forward during RTC-to-NTP correction.

A validity bit of `0` means the corresponding FFT slot must not be
treated as valid observation data, regardless of the bytes currently
present in that slot.

This avoids requiring the entire file to be pre-filled with NaN values
when a file is created.

## 9. RTC and NTP time handling

Tab5-HRO can operate without network access using its RTC.

When NTP later becomes available, system time may move forward or
backward relative to RTC-derived time.

To handle this safely, the internal save queue record should contain a
timestamp even though per-record timestamps are not written to the
`.hro` file.

Conceptually:

``` cpp
struct HroSaveRecord
{
    time_t timestamp;
    float spectrum[HRO_BIN_COUNT];
};
```

The SD writer derives the destination file and second slot from this
timestamp.

### 9.1 Backward time correction

If NTP moves time backward and the newly calculated slot has already
been written, the existing slot must not be overwritten.

The event should be counted as a time conflict.

Recommended header information includes:

``` text
START_TIME_SOURCE=RTC
NTP_SYNCED=1
NTP_SYNC_TIME_UTC=...
TIME_CONFLICTS=...
```

### 9.2 Forward time correction

If NTP moves time forward, skipped second slots remain invalid in the
validity map.

### 9.3 Hour boundary correction

If corrected time belongs to another clock hour, the SD writer selects
the file corresponding to the corrected timestamp.

## 10. SD-card writing architecture

SD-card access must not occur directly in the SDR sampling, DSP, or
time-critical FFT processing path.

Recommended architecture:

``` text
SDR / DSP / FFT
       |
       | one completed spectrum per second
       v
non-blocking Save Queue
       |
       v
SD Writer Task
       |
       v
SD card
```

The producer must use a non-blocking queue operation.

If the queue is full, the observation record may be dropped from storage
rather than blocking the DSP/FFT pipeline.

A separate storage-drop counter should be maintained.

The SD writer may buffer multiple seconds in RAM/PSRAM and write them in
batches to reduce filesystem and SD-card transaction overhead.

The primary design rule is:

> Loss or delay in data storage must not cause loss or delay in SDR
> sampling or DSP processing.

## 11. Header finalization

Some header values are not known when the file is first created, for
example:

``` text
NTP_SYNCED
NTP_SYNC_TIME_UTC
TIME_CONFLICTS
```

An implementation may initially write provisional values and update the
fixed-size header when the file is closed or when the hour changes.

Header updates must not change `HEADER_SIZE` or move the binary
sections.

For crash tolerance, readers should treat the validity map as the
authoritative indication of which second slots contain valid data.

## 12. Compatibility rules

Readers of Version 1 should:

-   verify `HRO_FILE_VERSION`;
-   use `HEADER_SIZE`;
-   use `VALIDITY_MAP_SIZE`;
-   use `FFT_BIN_COUNT`;
-   use `DATA_TYPE` and `BYTE_ORDER`;
-   ignore unknown text-header keys;
-   consult the validity map before using an FFT slot;
-   not assume that every one of the 3600 slots is valid.

Writers must not silently change the meaning or binary representation of
an existing Version 1 field. Incompatible changes require a new
`HRO_FILE_VERSION`.

## 13. Version 1 items to finalize before public specification freeze

The following details should be explicitly finalized before declaring
the format stable:

1.  exact validity-map bit ordering;
2.  exact FFT amplitude definition/normalization and unit;
3.  mandatory versus optional header keys;
4.  handling and escaping of newline or `=` characters in free-text
    metadata;
5.  exact padding byte used for the unused text-header area;
6.  software/file-format versioning policy;
7.  behavior for duplicate timestamps and storage queue drops;
8.  whether header fields updated at close require a crash-safe
    secondary mechanism.

Until these items are frozen, this document should be treated as a draft
specification.

------------------------------------------------------------------------

Copyright / license terms for this file-format specification may be
added by the Tab5-HRO project before public distribution.
