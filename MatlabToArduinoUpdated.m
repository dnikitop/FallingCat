
clc
clear

%the script uses fopen and fclose to connect to arduino
%if you manually exit with ctrl-c, make sure you fclose(s), or you will
%have to start matlab over again

%input motor values here, also edit COM port for the arduino
% make sure they are the same length
%load DataExp
%load LOAD_Arduino_CMD
load GetData
pm1
pm2
%pm1 = [25,25,25,25,25,25,25,25,25,25,25,25,26,26,26,26,26,27,27,27,28,28,29,29,30,30,31,31,32,32,33,34,35,35,36,37,38,39,39,40,41,42,43,44,45,46,48,49,50,51,52,53,55,56,57,59,60,61,63,64,65,67,68,70,71,72,74,75,77,78,79,81,82,84,85,86,88,89,90,92,93,94,96,97,98,99,100,101,103,104,105,106,107,107,108,109,110,111,111,112,113,113,114,114,115,115,116,116,116,116,116,117,117,117,117,117,116,116,116,116,115,115,115,114,114,113,113,112,111,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,95,94,93,91,90,89,87,86,85,83,82,80,79,78,76,75,73,72,70,69,68,66,65,63,62,61,59,58,57,55,54,53,52,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,36,35,34,33,33,32,31,31,30,30,29,29,28,28,27,27,27,26,26,26,26,26,25,25,25,25,25,25,25,25,25,25,25,25];
%pm2 = [90,90,90,90,90,90,90,90,91,91,91,91,92,92,93,93,93,94,94,95,96,96,97,98,98,99,100,101,101,102,103,104,105,106,107,107,108,109,110,111,112,113,114,115,116,117,118,118,119,120,121,122,122,123,124,124,125,126,126,127,127,128,128,128,129,129,129,129,130,130,130,130,130,129,129,129,129,128,128,128,127,127,126,125,125,124,123,122,122,121,120,119,118,117,115,114,113,112,111,109,108,107,106,104,103,101,100,99,97,96,94,93,91,90,88,87,85,84,83,81,80,78,77,76,74,73,72,70,69,68,67,65,64,63,62,61,60,59,58,57,57,56,55,55,54,53,53,52,52,52,51,51,51,51,51,51,51,51,51,51,51,51,52,52,52,53,53,54,54,55,56,56,57,58,58,59,60,61,62,62,63,64,65,66,67,68,69,70,71,71,72,73,74,75,76,77,78,79,79,80,81,82,82,83,84,84,85,86,86,87,87,87,88,88,89,89,89,89,90,90,90,90,90,90,90,90];
s = serial('COM12','BaudRate',115200); 

%pm1(1,77:130) = 25;
%pm2(1,77:130) = 90;

pm1(1,39:50) = 25;
pm2(1,39:50) = 90;
%{ 
rm1;
rm2;

gyrox;
gyroy;t

gyroz;
t
accelx;
accely;
accelz;
%}
len = length(pm1)
pause on

fopen(s);
pause(2);
z = 0;
a = [];
while(z == 0)
    prompt = 'press t to transmit, m to move, or e to exit: ';
    x = input(prompt,'s');
    if (x == 't')
        output = 1;
        fwrite(s,output,'uchar');
        pause(1);
        transmit(s,pm1,pm2,len);
    elseif (x == 'a')
        output = 2;
        flushinput(s);
        fwrite(s,output,'uchar');
        while(s.BytesAvailable <14)
            display(s.BytesAvailable)
        end
        accel = fread(s,14,'uchar')
    elseif (x == 'd')     
        output = 3;
        fwrite(s,output,'uchar');
        a = getFeed(s);
    elseif (x == 'm')     
        output = 4;
        fwrite(s,output,'uchar');
        getNoFeed(s);
    elseif (x == 'e')
        z = 1;
    end
end


fclose(s);

function a = transmit(s,pm1,pm2,len)
    for index = 1:10:len-9
        pause(.3);
        out = [250,0,len,pm2(1,index:index+9),251];
        fwrite(s,out,'uchar');
    end
    out = [250,11,len,0,0,0,0,0,0,0,0,0,0,251];
    fwrite(s,out,'uchar');
    for index = 1:10:len-9
        pause(.1);
        out = [250,0,len,pm1(1,index:index+9),251];
        fwrite(s,out,'uchar');
    end
    pause(.3);
    out = [250,10,len,0,0,0,0,0,0,0,0,0,0,251];
    fwrite(s,out,'uchar');
    
end
    
function totalinput = getFeed(s)
qq = 1;
totalinput = []
while qq == 1
    while(s.BytesAvailable < 14)
    end
    
    input = fread(s,14);
    if (input(2,1) == 10)
        qq = 0;
    end
    totalinput = [totalinput,input];
    input
end
end

function totalinput = getNoFeed(s)
qq = 1;
while qq == 1
    while(s.BytesAvailable < 14)
    end
    input = fread(s,14);
    if (input(2,1) == 10)
        qq = 0;
    end
end
end
      
