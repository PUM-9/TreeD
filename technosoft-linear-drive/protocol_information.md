# Purpose

This file contains information about the hardware powering the linear drive. The
information contained in this file should be enough to properly implement the
software needed.

# The hardware 

The hardware powering the linear drive is a servo made by [technosoft]
(http://technosoftmotion.com/en/). It is programmed using their proprietary
assemply language TML. They have developed a SDK called EasyMotion Studio used
for programming their hardware. 

# Old implementation

The old implementation is based on sending uncommented hexadecimal data to the
servo. Yay.

## Serial communication

Here we describe the serial communication used in the old implementation

### Opening
* Baudrate 115200
* Just open and send as usual.

### Sync

The old implementation contains a function called `fvTechSoftSync`,
```
function out = fvTechSoftSync(s)

out = 1;

for i = 1:15
	fwrite(s,13,'uint8');
    status = fread(s,1,'uint8');
    if status==13
        out = 0;
        return;
    end
end
```
, which simply seems to dump `13` at the serial port until it returns `13`.

### Move

Here be dragons.

```
function fvTechSoftMove(s,gohome,vel,acc)

if nargin>2
    error('Not implemented')
end
if nargin<1
    error('Missing serial object')
end
if nargin < 2 | isempty(gohome)
    gohome=1;
end

cmdHexStr{1}='08 0F F0 24 A2 00 55 00 00 22 ';
cmdHexStr{2}='08 0F F0 24 A0 8C 9F 00 09 FF ';
if gohome == 0
    % minus
    %            '08 00 10 24 9E 80 00 FF FF 58 ' = -32768
    cmdHexStr{3}='08 0F F0 24 9E 4C D0 FF FE E2 '; %lång
    %cmdHexStr{3}='08 0F F0 24 9E C4 00 FF FE 8A ';
else
    %            '08 0F F0 24 9E 00 01 00 00 XX ' = 1
	%            '08 0F F0 24 9E 00 FF 00 00 XX ' = 255
	%            '08 0F F0 24 9E 01 00 00 00 XX ' = 256
    %            '08 00 10 24 9E 80 00 00 00 5A ' = 32768
    %            '08 0F F0 24 9E FF FF 00 00 XX ' = 65535
    %            '08 0F F0 24 9E 00 00 00 01 CA ' = 65536
    cmdHexStr{3}='08 0F F0 24 9E B3 30 00 01 AD '; %lång
    %cmdHexStr{3}='08 0F F0 24 9E 3C 00 00 01 06 '; %
end
cmdHexStr{4}='08 0F F0 59 09 DF FF 00 00 47 ';
cmdHexStr{5}='08 0F F0 59 09 BF C1 87 01 71 ';
cmdHexStr{6}='08 0F F0 59 09 FF FF 40 00 A7 ';
cmdHexStr{7}='04 0F F0 01 08 0C ';
cmdHexStr{8}='04 0F F0 70 0F 82 04 0F F0 04 08 0F ';

c = 1;
while (c<=length(cmdHexStr))
    cmd = [];
    for v = 1:length(cmdHexStr{c})/3
        cmd = [cmd hex2dec(cmdHexStr{c}(3*v-2:3*v-1))];
    end
    fwrite(s,cmd,'uint8');
    status = fread(s,1,'uint8');
    if status ~= 79
        sync_count = 0;
        while (fvTechSoftSync(s) && (sync_count < 10))
           pause(0.1);
           sync_count = sync_count + 1;
        end
        if sync_count >= 10
            error('Can''t sync. Sorry.')
        end
    else
        c = c + 1;
    end
	pause(0.1)
end

status = fread(s,1,'uint8');
if status == 13
    % Needs resync
    sync_count = 0;
    while (fvTechSoftSync(s) && (sync_count < 10))
       pause(0.1);
       sync_count = sync_count + 1;
    end
    if sync_count >= 10
        error('Can''t sync. Sorry.')
    end
    disp('Re-synced!');
elseif status ~=79
    disp(sprintf('Recieved status %d', status));
    error('There was something wrong with the last transfer.')
end
```

The actual data that sent is
```
cmdHexStr{1}='08 0F F0 24 A2 00 55 00 00 22 ';
cmdHexStr{2}='08 0F F0 24 A0 8C 9F 00 09 FF ';
cmdHexStr{3}='08 0F F0 24 9E 4C D0 FF FE E2 '; %lång <- this seems to contain the actual movement target
cmdHexStr{4}='08 0F F0 59 09 DF FF 00 00 47 ';
cmdHexStr{5}='08 0F F0 59 09 BF C1 87 01 71 ';
cmdHexStr{6}='08 0F F0 59 09 FF FF 40 00 A7 ';
cmdHexStr{7}='04 0F F0 01 08 0C ';
cmdHexStr{8}='04 0F F0 70 0F 82 04 0F F0 04 08 0F ';
```
, however, if the flag `gohome` is set `cmdHexStr{3}` is set as
```
cmdHexStr{3}='08 0F F0 24 9E B3 30 00 01 AD ';
```
.
#### Sending the data

The data is dumped at the serial port one line at a time. It seems like the
controller will return `79 (==0x4f)` if the transfer is successful. Otherwise a
resync and resend of the command seems to be nessecary. The rest of the code in
the loop is simply for bailing out if we cannot resync.


