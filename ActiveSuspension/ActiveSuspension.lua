--This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published
--by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
--This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
--You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
--Copyright StiffBoards 2023

--PLEASE SEE APPLICABLE LICENSE in https://github.com/StiffBoards/myPlasmaScripts/blob/main/ActiveSuspension/COPYING.TXT

--ABANDON ALL HOPE ALL YE WHO ENTER
--left actuators negative, right actuators positive
--steering actuators all positive
--single axis joint is 0.25 meters long
--wheel width 1.6973
--add 0.0177 to distance sensor output, offset from center
----game offset 100% is 0.1
function setup()
initTimer=0----need a timer because SOME SENSOR VALUES START OUT NIL AND MESS EVERYTHING UP!
dt=1/30
debug=""
deg2rad=math.pi/180
worldGravities={earth=-9.81,moon=-1.62,mars=-3.72,space=0.0,australia=9.81}
world=read_var("aworld")--get the current world so we can look up gravity
gravAccel=-worldGravities[world]
legLeng=(0.875+0.0280-0.0625+0.55*0.5*0.25)--length from center of motor to linkage actuator
linkLeng=(0.0625+0.55*0.25*0.5)--length of linkage
wheelD=1*0.5
strLeng=0.5845-(0.5*0.25*0.45)-0.5*wheelD---distance from center of steering link to center of wheel
strCoM=(((0.1/2)-0.28*0.1)*2)/(2+0.5)---center of mass of steering link
strMass=2.5
---strLeng=0

--nnComp=0.003--non-newtonian compensation, compensate for momentum not being conserved
nnComp=0.00002
wBase=1.6973-wheelD
xOffset=0.25+0.1-0.6*0.1+0.5*0.125+0.45*0.25-0.41*0.1
zOffset=0.0947--z distance of leg attachment point from origin
right={0,0,0}
forward={0,0,0}
up={0,0,0}
corePitch=0
coreRoll=0
angVel={0,0,0}
localVel={0,0,0}
lastVel={0,0,0}--last world velocity
worldAccel={0,0,0}
accel={0,0,0}
measuredH=0
bodyMass=31.32
bodyCoM={0,0.15625,0}
blCom={0,0,0}
brCom={0,0,0}
flCom={0,0,0}
frCom={0,0,0}
blPos={0,0,0}
brPos={0,0,0}
flPos={0,0,0}
frPos={0,0,0}
screenScale=450/2
screenScale=300/2
write_var(screenScale*0.5*wheelD,"cirrad")
--bodyMass=25
pitchAmt=30*deg2rad
rollAmt=30*deg2rad
--attitude relative to terrain plane
rollP=0
pitchP=0
enable=true
targetAlt=0
targetAltMode=false
lastAct1=false
targetH=0.35--should be sketch variable
torqLim=1000
targetPitch=0
targetRoll=0
maxDist=20--max distance reported by distance sensor
minForce=0
heaveGain=3000
--heaveGain=0
azGain=1
axGain=90
--100
--axGain=50
yDamp=0
rollInt=0
--axGain=0
--linkage angles
bllinkr=0
brlinkr=0
fllinkr=0
frlinkr=0
--actuator angles
blactr=0
bractr=0
fractr=0
flactr=0
jump=false
strcmd=0
frstr=0
flstr=0
fldrs=0
bldrs=0
frdrs=0
brdrs=0
rollWF=HPF:create(0.98)
contMax=0.3
contSt=0.2--contact start

yRDamp=35
yK=500
yBias=0
hBias=0.2
end
function toScreenCoords(vec2,scale)
return {math.floor(340+vec2[1]*scale),math.floor(256+vec2[2]*scale)}
end
function pitchRollTarget()
  
  local heaveE= targetH-measuredH
  if targetAltMode then
  heaveE=targetAlt-origin[2]
  end
  local yvel=worldVel[2]
heaveE=math.max(heaveE,-0.5)
  --local heaveF= PIDctrl(heavePID,heaveE)-80*yvel
--400 times y vel
 coreUp=dot({0,1,0},up)
 gravComp=gravAccel*(0+bodyMass)/coreUp

  heaveF=gravComp+heaveGain*heaveE-400*yvel- 1*accel[3]--seems to work better on very rough terrain
  local pitchE=corePitch-targetPitch
  local rollE=targetRoll-coreRoll
  if not(read_var("pitchstabmode")) then
    pitchE=pitchP-targetPitch
  end
  ---if not(rollStabMode) then
  --  rollE=rollP+targetRoll
  --end

  local pitchF=200*pitchE-50*angVel[1]
--800
 -- local rollF =700*rollE-74*(angVel[2])+0*rollInt
 local rollF =700*rollE-90*rollWF:update(angVel[2])+0*rollInt
-- local rollF =800*rollE-150*angVel[2]+1*rollInt
 rollInt=math.clamp(rollE+rollInt,-math.pi,math.pi)
  if read_var("jump") then
    heaveF=10000
  end
--heaveE
output(accel[1],7)
addToLog(string.format("h %1.3f p %1.3f r %1.3f %1.3f",heaveF,pitchF,rollF,measuredH))
---heaveF=0
susActTorque(heaveF,-pitchF,rollF) 
end

function rot2d(vec2,ang)
return {vec2[1]*math.cos(ang)-vec2[2]*math.sin(ang),vec2[1]*math.sin(ang)+vec2[2]*math.cos(ang)}
end
function generateGraphics()
---determine where wheels are
---project down
--- output an array of points where the wheels are
--- calculate CG location

---convert wheels to world coords
lblpos=local2world(right,up,forward,blPos)
lbrpos=local2world(right,up,forward,brPos)
lflpos=local2world(right,up,forward,flPos)
lfrpos=local2world(right,up,forward,frPos)
---projects to screen
lblsc=toScreenCoords(rot2d({lblpos[1],lblpos[3]},coreYaw),screenScale)
lbrsc=toScreenCoords(rot2d({lbrpos[1],lbrpos[3]},coreYaw),screenScale)
lflsc=toScreenCoords(rot2d({lflpos[1],lflpos[3]},coreYaw),screenScale)
lfrsc=toScreenCoords(rot2d({lfrpos[1],lfrpos[3]},coreYaw),screenScale)
write_var(lblsc[1],"cirblx")
write_var(lblsc[2],"cirbly")
write_var(lbrsc[1],"cirbrx")
write_var(lbrsc[2],"cirbry")
write_var(lflsc[1],"cirflx")
write_var(lflsc[2],"cirfly")
write_var(lfrsc[1],"cirfrx")
write_var(lfrsc[2],"cirfry")
lineout={lblsc[1],lblsc[2],lbrsc[1],lbrsc[2],lfrsc[1],lfrsc[2],lflsc[1],lflsc[2],lblsc[1],lblsc[2]}
output_array(lineout,2)
--calculate cg location
com=vadd(vmul(flCom,strMass),vmul(frCom,strMass))
com=vadd(com,vmul(brCom,strMass))
com=vadd(com,vmul(blCom,strMass))
com=vadd(com,vmul(bodyCoM,bodyMass))
com=vmul(com,1/(4*strMass+bodyMass))
wcom=local2world(right,forward,up,com)
cgsc=toScreenCoords(rot2d({wcom[1],wcom[3]},coreYaw),screenScale)
write_var(cgsc[1]-25,"cgx")
write_var(cgsc[2]-19,"cgy")
write_var(math.min(100*(blDist-contSt)/contMax,100),"cirblc")
write_var(math.min(100*(brDist-contSt)/contMax,100),"cirbrc")
write_var(math.min(100*(flDist-contSt)/contMax,100),"cirflc")
write_var(math.min(100*(frDist-contSt)/contMax,100),"cirfrc")
trigger(1)
end



function magic(al,ar,bl,br,w,fZ,fPi,fRol)
--High level dark magic, you have to be a wizard to understand this
local c1div=(6.0*al^2 + 12.0*al*bl + 6.0*ar^2 + 12.0*ar*br + 6.0*bl^2 + 6.0*br^2)
local c2div=c1div
local c3div=(w*(4.0*al^2 + 8.0*al*bl + 4.0*ar^2 + 8.0*ar*br + 4.0*bl^2 + 4.0*br^2))
local w11=(-1.5*al*ar + 3.0*al*bl + 1.5*al*br + 1.5*ar^2 - 1.5*ar*bl + 3.0*ar*br + 3.0*bl^2 + 1.5*bl*br + 1.5*br^2)/c1div
local w21=(1.5*al^2 - 1.5*al*ar + 3.0*al*bl - 1.5*al*br + 1.5*ar*bl + 3.0*ar*br + 1.5*bl^2 + 1.5*bl*br + 3.0*br^2)/c1div
local w31=(3.0*al^2 + 1.5*al*ar + 3.0*al*bl - 1.5*al*br + 1.5*ar^2 + 1.5*ar*bl + 3.0*ar*br - 1.5*bl*br + 1.5*br^2)/c1div
local w41=(1.5*al^2 + 1.5*al*ar + 3.0*al*bl + 1.5*al*br + 3.0*ar^2 - 1.5*ar*bl + 3.0*ar*br + 1.5*bl^2 - 1.5*bl*br)/c1div
local w12=-(6.0*al + 6.0*bl)/c2div
local w22=-(6.0*ar + 6.0*br)/c2div
local w32=6.0*(al + bl)/c2div
local w42=6.0*(ar + br)/c2div
local w13=(2.0*al*ar + 4.0*al*bl - 2.0*al*br + 2.0*ar^2 + 2.0*ar*bl + 4.0*ar*br + 4.0*bl^2 - 2.0*bl*br + 2.0*br^2)/c3div
local w23=(-2.0*al^2 - 2.0*al*ar - 4.0*al*bl - 2.0*al*br + 2.0*ar*bl - 4.0*ar*br - 2.0*bl^2 + 2.0*bl*br - 4.0*br^2)/c3div
local w33=(4.0*al^2 - 2.0*al*ar + 4.0*al*bl + 2.0*al*br + 2.0*ar^2 - 2.0*ar*bl + 4.0*ar*br + 2.0*bl*br + 2.0*br^2)/c3div
local w43=(-2.0*al^2 + 2.0*al*ar - 4.0*al*bl + 2.0*al*br - 4.0*ar^2 - 2.0*ar*bl - 4.0*ar*br - 2.0*bl^2 - 2.0*bl*br)/c3div

local IDT={{w11,w12,w13},{w21,w22,w23},{w31,w32,w33},{w41,w42,w43}}
IDT.r=4
IDT.c=3
local fIn={{fZ},{fPi},{fRol}}
fIn.r=3
fIn.c=1
wf=matrixMul(IDT,fIn)
return wf --wheel forces in {{ffl},{ffr},{fbl},{fbl}} and are oriented up
end
--high pass filter
HPF = {}
HPF.__index = HPF
function HPF:create(rc)
   local nf = {}             
   setmetatable(nf,HPF)  
   nf.rc = rc
   nf.alpha=rc/(rc+(1/30))
   nf.x=0
   nf.y=0      
   return nf
end

function HPF:update(input)
self.y=self.alpha*(self.y+input-self.x)
self.x=input
return self.y
end

WOF = {}
WOF.__index = WOF
function WOF:create(rc)
   local nf = {}             
   setmetatable(nf,HPF)  
   nf.rc = rc
   nf.alpha=1/(rc+(1/30))
   nf.x=0
   nf.y=0      
   return nf
end

function WOF:update(input)
self.y=self.alpha*(self.y+input-self.x)
self.x=input
return self.y
end

function loop()
debug=""
foo=math.clamp(10,1,2)
updateBaseProperties()
if initTimer<10 then 

initTimer=initTimer+1
else

pitchRollTarget()
--output(measuredH,7)
end
output(debug,8)
end
function susActTorque(fZ,fPi,fRol)
ar=math.abs(frz)
al=math.abs(flz)
br=math.abs(brz)
bl=math.abs(blz)
whForces = magic(al,ar,bl,br,wBase,fZ,fPi,fRol)
--accel[1]=10
---accelX=accel[1]*math.cos(-coreRoll)+accel[3]*math.sin(-coreRoll)
--accelX=dot(worldAccel,normalize({right[1],right[2],0}))
accelX=accel[1]
local latAXDelt=axGain*(accelX)*(targetH/wBase)/gravAccel
--local latAXDelt=0
--2
--mComp=nnComp*read_var("drivecmd")
local nomWL=2*(zOffset+linkLeng+math.sqrt(legLeng^2-targetH^2))
local latAZDelt=azGain*(targetH/nomWL)*accel[2]/gravAccel
local gtfl=lafl*(whForces[1][1]+latAXDelt +latAZDelt-yDamp*legLeng*math.sin(flactrs))+nnComp*fldrs---+latAXDelt)
local gtfr=-lafr*(whForces[2][1] -latAXDelt +latAZDelt-yDamp*legLeng*math.sin(fractrs)) -nnComp*frdrs-- -latAXDelt)
local gtbl=labl*(whForces[3][1] +latAXDelt -latAZDelt-yDamp*legLeng*math.sin(blactrs))+nnComp*bldrs--+latAXDelt)
local gtbr=-labr*(whForces[4][1] -latAXDelt -latAZDelt-yDamp*legLeng*math.sin(bractrs))-nnComp*brdrs-- -latAXDelt)
if not( read_var("susmode")) then

latAZDelt=0
latAXDelt=0

targetSusHeight=-(targetH+hBias-wheelD/2)
if read_var("jump") then
targetSusHeight=-25
end
sprFl=-yK*(targetSusHeight-flPos[2])+yBias
sprFr=-yK*(targetSusHeight-frPos[2])+yBias
sprBl=-yK*(targetSusHeight-blPos[2])+yBias
sprBr=-yK*(targetSusHeight-brPos[2])+yBias
gtfl=lafl*(sprFl+latAXDelt +latAZDelt-yRDamp*legLeng*math.sin(flactrs))+nnComp*fldrs---+latAXDelt)
 gtfr=-lafr*(sprFr-latAXDelt +latAZDelt-yRDamp*legLeng*math.sin(fractrs)) -nnComp*frdrs-- -latAXDelt)
 gtbl=labl*(sprBl+latAXDelt -latAZDelt-yRDamp*legLeng*math.sin(blactrs))+nnComp*bldrs--+latAXDelt)
gtbr=-labr*(sprBr-latAXDelt -latAZDelt-yRDamp*legLeng*math.sin(bractrs))-nnComp*brdrs-- -latAXDelt)
end
--ik gains
local kp=1000
local kd=100
torqControlMotor("blact",gtbl)
torqControlMotor("bract",gtbr)
torqControlMotor("flact",gtfl)
torqControlMotor("fract",gtfr)
generateGraphics()
output(string.format("fl %1.0f fr %1.0f\n bl %1.0f br %1.0f",-gtfl,gtfr,gtbl,-gtbr),6)
addToLog(string.format("fr %1.3f fl %1.3f br %1.3f bl %1.3f",gtfr,gtfl,gtbr,gtbl))
--addToLog(string.format("fr %1.3f fl %1.3f br %1.3f bl %1.3f",fractrs, flactrs,bractrs,blactrs))
end

function torqControlMotor(name,torq)
torq=math.clamp(torq,-torqLim,torqLim)
---this is where the magic happens
if enable then
write_var(math.abs(torq),name.."trq")
else
write_var(0,name.."trq")
end

motorVel=100000--should be an unattainable value
if torq>0 then 
write_var(motorVel,name.."v")
else
write_var(-motorVel,name.."v")
end
end
function updateBaseProperties()
targetH=read_var("targeth")
gForward=read_var("gpsforward")
gRight=read_var("gpsright")
gLeft=read_var("gpsleft")
worldVel=read_var("gpsoriginvel")
targetPitch=read_var("pitchcmd")*pitchAmt/100
targetRoll=read_var("rollcmd")*rollAmt/100
origin=vmul(vadd(gRight,gLeft),0.5)
right=normalize(vsub(gRight,origin))
forward=normalize(vsub(gForward,origin))
up=normalize(vcross(forward,right))
addToLog(string.format("up x %1.3f y %1.3f z %1.3f",up[1],up[2],up[3]))
--calculate local accleration because accelerometer is too big
--acceleration's fake anyway
localVel=world2local(right,forward,up,worldVel) 
worldAccel=vmul(vsub(worldVel,lastVel),1/dt)
accel=world2local(right,forward,up,worldAccel) 
lastVel=worldVel

gyro=read_var("gyroang")
angVel=vmul(read_var("gyroRate"),deg2rad)
corePitch=deg2rad*gyro[1]
coreRoll=-deg2rad*gyro[2]
coreYaw=deg2rad*gyro[3]
midDist=processDistSensor("middist","middistd")
blDist=processDistSensor("bldist","bldistd")
brDist=processDistSensor("brdist","brdistd")
flDist=processDistSensor("fldist","fldistd")
frDist=processDistSensor("frdist","frdistd")
measuredH=midDist*math.cos(math.atan(math.sqrt(math.tan(corePitch)^2+math.tan(coreRoll)^2)))
--oh boy here we go
blactr=-read_var("blactr")*deg2rad
blactrs=-read_var("blactrs")*deg2rad
bractr=read_var("bractr")*deg2rad
bractrs=read_var("bractrs")*deg2rad
fractr=read_var("fractr")*deg2rad
fractrs=-read_var("fractrs")*deg2rad
flactr=read_var("flactr")*deg2rad
--negative?
flactrs=read_var("flactrs")*deg2rad

bllinkr=read_var("bllinkr")*deg2rad
brlinkr=-read_var("brlinkr")*deg2rad
fllinkr=read_var("fllinkr")*deg2rad
frlinkr=read_var("frlinkr")*deg2rad
act1=read_var("action1")
strcmd=read_var("strcmd")*deg2rad
frstr=read_var("frstr")*deg2rad
flstr=read_var("flstr")*deg2rad
frdrs=read_var("frdrs")*deg2rad
fldrs=-read_var("fldrs")*deg2rad
brdrs=read_var("brdrs")*deg2rad
bldrs=-read_var("bldrs")*deg2rad
if not(act1) and lastAct1 then
targetAltMode=not(targetAltMode)
targetAlt=(targetH-measuredH)+origin[2]
end
lastAct1=act1
--link orientation
blo=(blactr+bllinkr)
blz=-(legLeng*math.cos(blactr)+linkLeng*math.cos(blo))
labl=blz
blz=blz-zOffset
bly=-(legLeng*math.sin(blactr)+linkLeng*math.sin(blo))
blx=-strLeng-xOffset
blPos={blx,bly,blz}

blCom={-strCoM-xOffset,bly,blz}
blod=math.pi/2+blo
bldz=blz-blDist*math.cos(blod)
bldy=bly-blDist*math.sin(blod)
--position of distance sensor point
blDp={blx,bldy,bldz}
bro=bractr+brlinkr
brz=-(legLeng*math.cos(bractr)+linkLeng*math.cos(bro))
labr=brz
brz=brz-zOffset
bry=-(legLeng*math.sin(bractr)+linkLeng*math.sin(bro))
brx=strLeng+xOffset

brPos={brx,bry,brz}
brCom={strCoM+xOffset,bry,brz}
brod=math.pi/2+bro
brdz=blz-blDist*math.cos(brod)
brdy=bly-brDist*math.sin(brod)
--position of distance sensor point
brDp={brx,brdy,brdz}

flo=(flactr+fllinkr)
flz=(legLeng*math.cos(flactr)+linkLeng*math.cos(flo))-strLeng*math.sin(flstr)
lafl=flz
flz=flz+zOffset
fly=-(legLeng*math.sin(flactr)+linkLeng*math.sin(flstr))
flx=-xOffset-strLeng*math.cos(flo)
flPos={flx,fly,flz}
flcz=(legLeng*math.cos(flactr)+linkLeng*math.cos(flo))-strCoM*math.sin(flstr)+zOffset
flcx=-xOffset-strCoM*math.cos(flo)
flCom={flcx,fly,flcz}
flod=math.pi/2+flo
fldz=flz-flDist*math.cos(flod)
fldy=fly-flDist*math.sin(flod)
--position of distance sensor point
flDp={flx,fldy,fldz}

fro=(fractr+frlinkr)
frz=(legLeng*math.cos(fractr)+linkLeng*math.cos(fro))+strLeng*math.sin(frstr)
lafr=frz
frz=frz+zOffset
fry=(legLeng*math.sin(fractr)+linkLeng*math.sin(fro))
frx=xOffset+strLeng*math.cos(frstr)
frPos={frx,fry,frz}
frcz=(legLeng*math.cos(fractr)+linkLeng*math.cos(fro))+strCoM*math.sin(frstr)+zOffset
frcx=xOffset+strCoM*math.cos(frstr)
frCom={frcx,fry,frcz}
frod=math.pi/2+flo
frdz=frz-frDist*math.cos(frod)
frdy=fry-frDist*math.sin(frod)
--position of distance sensor point
frDp={frx,frdy,frdz}

--terrain plane estimation
local normBR=vcross(vsub(frDp,brDp),vsub(blDp,brDp))
local normFR=vcross(vsub(flDp,frDp),vsub(brDp,frDp))
local normFL=vcross(vsub(brDp,flDp),vsub(frDp,flDp))
local normBL=vcross(vsub(brDp,blDp),vsub(flDp,blDp))
cNorm=normalize(vadd(normBR,vadd(normBL,vadd(normFL,normFL))))
rollP=-math.atan2(cNorm[1],-cNorm[2])
pitchP=math.atan2(cNorm[3],-cNorm[2])

--addToLog(string.format("bl: a: %1.3f l %1.3f o %1.3f",blactr,bllinkr,blo))
addToLog(string.format("bl: z %1.3f, y %1.3f",blz,bly))
--addToLog(string.format("br: a: %1.3f l %1.3f o %1.3f",bractr,brlinkr,bro))
addToLog(string.format("br: z %1.3f, y %1.3f",brz,bry))
--addToLog(string.format("fl: a: %1.3f l %1.3f o %1.3f",flactr,fllinkr,flo))
addToLog(string.format("fl: z %1.3f, y %1.3f",flz,fly))
--addToLog(string.format("fr: a: %1.3f l %1.3f o %1.3f",fractr,frlinkr,fro))
addToLog(string.format("flr z %1.3f, y %1.3f",frz,fry))
end
function addToLog(text)
debug=debug..text.."\n"
end


function nil2vec(vec)
if vec == nil then
return {0,0,0}
else
return vec
end
end
function printVec(vec)
return string.format("%2.4f,%2.4f,%2.4f",vec[1],vec[2],vec[3])
end
function matrixMul(m1,m2)
--matrix multiplication, requires that matrices have a .r and .c for specifying number of rows and columns
        --if(m1.c~=m2.r)then
        --        error(string.format("m1:%d , m2:%d",m1.c,m2.r),2)
        --end
        local mat = {}
        mat.r = m1.r
        mat.c = m2.c
       
        for i=1,m1.r do
                mat[i] = {}
                for j=1,m2.c do
                        mat[i][j] = 0
                        for k=1,m1.c do
                                mat[i][j] = mat[i][j] + m1[i][k]*m2[k][j]
                        end
                end
        end
        return mat
end

function vadd(a,b)
return {a[1]+b[1],a[2]+b[2],a[3]+b[3]}
end
function vsub(a,b)
return {a[1]-b[1], a[2]-b[2], a[3]-b[3]}
end
function vmul(a,s)
--multiply vector by scalar
return {a[1]*s,a[2]*s,a[3]*s}
end
function vcross(a,b)
--cross product a x b
return {a[2]*b[3]-a[3]*b[2],a[3]*b[1]-a[1]*b[3],a[1]*b[2]-a[2]*b[1]}
end
function mag(a)
return math.sqrt(a[1]^2+a[2]^2+a[3]^2)
end

function normalize(a)
local mg=mag(a)
return {a[1]/mg,a[2]/mg,a[3]/mg}
end
function dot(a,b)
return a[1]*b[1]+a[2]*b[2]+a[3]*b[3]
end


function nan2Zero(a)
if a~=a then
return 0
else
return a
end
end

function world2local( right,forward,up,vec)
return {dot(right,vec),dot(forward,vec),dot(up,vec)}
end

function local2world(lright, lforward,lup, vec)
return vadd(vadd(vmul(lright,vec[1]),vmul(lforward,vec[2])), vmul(lup,vec[3]))
end
function processDistSensor(distVar,detectVar)
detect=nil2vec(read_var(detectVar))
if detect[1] == "nothing" or detect[1]==0  then
	return maxDist
else
	return read_var(distVar)
end

end
