# Primitive bootloader for stm32, that supports programming via Unified Diagnostic Services (ISO-14229)

## Supported services
|Service|Sub-function|Note|
|---|---|---|
|diagnosticSessionControl (0x10)|programmingSession (0x02)|Needs to enter in programmingSession before downloading software|
|ECUReset (0x11)|HardReset (0x01)|Performs reboot via watchdog|
|ReadDataByIdentifier (0x22)|bootSoftwareIdentificationDataIdentifier (0xF180)|Returns ASCII string|
|RequestDownload (0x34)|   |Performs erasing flash at this stage. dataFormatIdentifier - 0x00 or 0x01 addressAndLengthFormatIdentifier = 44|
|TransferData (0x36)|   |Transfers chunks of data, about 200 bytes|
|RequestTransferExit (0x37)|   |Writes to flash all buffers|
|TesterPresent (0x3E)|   |Do nothing|

## Example client software
[Python script](flash.py) - uses udsoncan and can-isotp modules
