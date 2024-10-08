

disp(' ')
disp('----Psystem Begins----')

x0s=[8, 80, 290, 300, 650,1,1,5000,0.9,16];
 
lb=[0.01,10,50,20,100,1,1,2600,0.01,20];
ub=[10, 100, 999, 999, 9999,300,30,8800,1,200];
A =[];
b = [];
Aeq = [];
beq = [];


constraints=@  mycon;
func=@ myfun;

options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',15) ;% ,'PlotFcn','optimplotfvalconstr');
%Run optimization for PS
[xopt,fvalopt, FLAG, OUTPUT] = fmincon(func,x0s,A,b,Aeq,beq,lb,ub,constraints,options);
xpdx = xopt;
dx = (ub(1) - lb(1))/100.0;
xpdx(1) = xopt(1) + dx
[xopt1,fvalopt, FLAG, OUTPUT] = fmincon(@(xpdx)  myfun_fix(xpdx,xpdx(1)), x0s, [], [], [], [], lb, ub, @(xpdx) mycon_fix(xpdx,xpdx(1)),options);      
    
      
%[~,sOUT]=myfun(xopt,xU,xL, vU,vL,wU,wL,misc);
[~,sOUT]=myfun(xopt);
sOUT.xopt=xopt;
sOUT.funcCount = OUTPUT.funcCount;
disp('_____Psystem  ends_________')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [foriginal,sOUT]=myfun(x)

%[rProp,V,mBattery,MMotor,
%mtow,Ereserve,S, rpm, eta_motor,m_gb]
    Pwing_output=Pwing(x(5),x(7));%2 outputs (mtow,ss)
    Pmotor_output=Pmotor(x(8),x(9)); % 2 outputs (rpm,eta_motorm)
    Pgearbox_output=Pgearbox(x(1),x(2),x(5),x(7),x(8),x(9),x(10)); % 7 outputs (rest 7 outputs)
    %Pprop_output=Pprop(x(1),x(2),x(3),x(4),x(6),x(10)); % 3 outputs
    %misc.range=rangei;
    rangei=50000;
    misc.range=rangei;
    misc.vehicle='tiltwing';
    misc.payload=300;
        rProp = x(1);
        V = x(2);
        mBattery = x(3);
        mMotors = x(4);
        mtow = x(5);
        S=x(7);
        rpm=x(8);
        eta_motor=x(9);
        m_gb=x(10);
        
        
        
        vehicle=misc.vehicle;
        range=misc.range;
        payload=misc.payload;
        
        
        
       
        % For the nominal mission compute the energy use, flight time, hover
        % performance, and cruise performance
        [ENominal,flightTime,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        
        
        % Mass estimate
        mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);
        
        % Compute operating cost
        C = operatingCost(vehicle,rProp,flightTime,ENominal,mass,cruiseOutput);
        
        %% Objective is operating cost per flight
        f = mtow;
        f=C.costPerFlight;
        
        
        
        %get original objective
        foriginal =f;
           
        % xsL=[mtoww,Sw,rpmm,eta_motorm,rPropg,Vg,mtowg,Sg,rpmg,eta_motorg,m_gbg];
        %Assemble c: target - response      
        c(1) = diffc(Pwing_output(1),mtow);  %from w
        c(2) = diffc(Pwing_output(2),S); %r
        c(3) = diffc(  Pmotor_output(1),rpm);   %w
        c(4) = diffc( Pmotor_output(2),eta_motor);  %w
        c(5) = diffc( Pgearbox_output(1),rProp);  %c
        c(6) = diffc(Pgearbox_output(2),V);    %w     
        c(7) = diffc(Pgearbox_output(3),mtow);  %w
        c(8) = diffc(Pgearbox_output(4),S);  %c
        c(9) = diffc(Pgearbox_output(5),rpm);    %w   
        c(10) = diffc(Pgearbox_output(6),eta_motor);    %w  
        c(11) = diffc(Pgearbox_output(7),m_gb);    %w  
        
        xL=[Pwing_output(1),Pwing_output(2),Pmotor_output(1),Pmotor_output(2),Pgearbox_output(1),Pgearbox_output(2),Pgearbox_output(3),Pgearbox_output(4),...,
            Pgearbox_output(5),Pgearbox_output(6),Pgearbox_output(7)];
        %Assemble v and w.Make it consistently ordered
        %get all phi
        %v = vL; w=wL;
        %phi= v.*c+(w.*c).^2;
        
        %ATC objective
        %fATC=foriginal/1e6+ sum(phi);
        
        %Pack all data to a struct
	    
        sOUT.rProp=rProp;
        sOUT.V = x(2);
        sOUT.mBattery = x(3);
        sOUT.mMotors = x(4);
        sOUT.mtow = x(5);
        sOUT.Ereserve = x(6);
        sOUT.S=x(7);
        sOUT.rpm=x(8);
        sOUT.eta_motor=x(9);
        sOUT.m_gb=x(10);
        
        
        sOUT.x = x;
        sOUT.xU = [];%xU is empty
        sOUT.xL = xL;
        %sOUT.vU = vU;
        %sOUT.vL = vL;
        %sOUT.wU = wU;
        %sOUT.wL = wL;
        sOUT.misc = misc;
        %sOUT.v = v;
        %sOUT.w = w;
        sOUT.c=c;
        %sOUT.phi=phi;
        sOUT.foriginal=foriginal;
        %sOUT.fATC = fATC;
        
        
    end
%Constraint
    function [cn,ceqn] = mycon(x)

    Pwing_output=Pwing(x(5),x(7));%2 outputs (mtow,ss)
    Pmotor_output=Pmotor(x(8),x(9)); % 2 outputs (rpm,eta_motorm)
    Pgearbox_output=Pgearbox(x(1),x(2),x(5),x(7),x(8),x(9),x(10)); % 7 outputs (rest 7 outputs)
    %Pprop_output=Pprop(x(1),x(2),x(3),x(4),x(6),x(10)); % 3 outputs
    %misc.range=rangei;
   
        rangei=50000;
        misc.range=rangei;
        misc.vehicle='tiltwing';
        misc.payload=300;
        rProp = x(1);
        V = x(2);
        mBattery = x(3);
        mMotors = x(4);
        mtow = x(5);
        Ereserve=x(6);
        S=x(7);
        rpm=x(8);
        eta_motor=x(9);
        m_gb=x(10);
        vehicle=misc.vehicle;
        range=misc.range;
        payload=misc.payload;

        ceqn(1) = diffc(Pwing_output(1),mtow);  %from w
        ceqn(2) = diffc(Pwing_output(2),S); %r
        ceqn(3) = diffc(  Pmotor_output(1),rpm);   %w
        ceqn(4) = diffc( Pmotor_output(2),eta_motor);  %w
        ceqn(5) = diffc( Pgearbox_output(1),rProp);  %c
        ceqn(6) = diffc(Pgearbox_output(2),V);    %w     
        ceqn(7) = diffc(Pgearbox_output(3),mtow);  %w
        ceqn(8) = diffc(Pgearbox_output(4),S);  %c
        ceqn(9) = diffc(Pgearbox_output(5),rpm);    %w   
        ceqn(10) = diffc(Pgearbox_output(6),eta_motor);    %w  
        ceqn(11) = diffc(Pgearbox_output(7),m_gb);    %w
        
        % Assumed values
        batteryEnergyDensity = 230; % Expected pack energy density in 3-5 years [Wh/kg]
        motorPowerDensity = 5;
        dischargeDepthReserve = 0.95; % Can only use 95% of battery energy in reserve mission
        
       [~,~,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        
        
        
        % Mass estimate
        mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);
        
        

        

        
        % Constraint on MTOW
        cn(1) = mass.W - mtow * 9.8;
        
        
        
        % Battery sizing
        [EReserve,~,~,~,~] = reserveMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        cn(2) = EReserve - mBattery * batteryEnergyDensity * dischargeDepthReserve / 1000;
        
%         % motor power density
%         cn(4) = hoverOutput.PMax / 1000 - mMotors * motorPowerDensity;
        
        
        %motor sizing based on torque
        torq=0.74*(hoverOutput.PMax/8)/(rpm*2*pi/60);
        lb2kg = 0.453592;   
        cn(3) = 0.3928*(torq^0.8587)*lb2kg*8 - mMotors;
        
        
%         % Constraint on tip speed
%         tip_mach=0.65;
%         omega=tip_mach*340.294/rProp;
%         rpm_max=omega*60/(2*pi);
%         cn(4)=rpm-rpm_max;
%         cn(4)=0;
        

        
        % Ereserve constraint
        ceqn(12)=abs (EReserve-x(6)); 
    end

%Objective
function [foriginal,sOUT]=myfun_fix(x,a)
x(1)=a;
%[rProp,V,mBattery,MMotor,
%mtow,Ereserve,S, rpm, eta_motor,m_gb]
    Pwing_output=Pwing(x(5),x(7));%2 outputs (mtow,ss)
    Pmotor_output=Pmotor(x(8),x(9)); % 2 outputs (rpm,eta_motorm)
    Pgearbox_output=Pgearbox(x(1),x(2),x(5),x(7),x(8),x(9),x(10)); % 7 outputs (rest 7 outputs)
    %Pprop_output=Pprop(x(1),x(2),x(3),x(4),x(6),x(10)); % 3 outputs
    %misc.range=rangei;
    rangei=50000;
    misc.range=rangei;
    misc.vehicle='tiltwing';
    misc.payload=300;
        rProp = x(1);
        V = x(2);
        mBattery = x(3);
        mMotors = x(4);
        mtow = x(5);
        S=x(7);
        rpm=x(8);
        eta_motor=x(9);
        m_gb=x(10);
        
        
        
        vehicle=misc.vehicle;
        range=misc.range;
        payload=misc.payload;
        
        
        
       
        % For the nominal mission compute the energy use, flight time, hover
        % performance, and cruise performance
        [ENominal,flightTime,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        
        
        % Mass estimate
        mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);
        
        % Compute operating cost
        C = operatingCost(vehicle,rProp,flightTime,ENominal,mass,cruiseOutput);
        
        %% Objective is operating cost per flight
        f = mtow;
        f=C.costPerFlight;
        
        
        
        %get original objective
        foriginal =f;
           
        % xsL=[mtoww,Sw,rpmm,eta_motorm,rPropg,Vg,mtowg,Sg,rpmg,eta_motorg,m_gbg];
        %Assemble c: target - response      
        c(1) = diffc(Pwing_output(1),mtow);  %from w
        c(2) = diffc(Pwing_output(2),S); %r
        c(3) = diffc(  Pmotor_output(1),rpm);   %w
        c(4) = diffc( Pmotor_output(2),eta_motor);  %w
        c(5) = diffc( Pgearbox_output(1),rProp);  %c
        c(6) = diffc(Pgearbox_output(2),V);    %w     
        c(7) = diffc(Pgearbox_output(3),mtow);  %w
        c(8) = diffc(Pgearbox_output(4),S);  %c
        c(9) = diffc(Pgearbox_output(5),rpm);    %w   
        c(10) = diffc(Pgearbox_output(6),eta_motor);    %w  
        c(11) = diffc(Pgearbox_output(7),m_gb);    %w  
        
        xL=[Pwing_output(1),Pwing_output(2),Pmotor_output(1),Pmotor_output(2),Pgearbox_output(1),Pgearbox_output(2),Pgearbox_output(3),Pgearbox_output(4),...,
            Pgearbox_output(5),Pgearbox_output(6),Pgearbox_output(7)];
        %Assemble v and w.Make it consistently ordered
        %get all phi
        %v = vL; w=wL;
        %phi= v.*c+(w.*c).^2;
        
        %ATC objective
        %fATC=foriginal/1e6+ sum(phi);
        
        %Pack all data to a struct
	    
        sOUT.rProp=rProp;
        sOUT.V = x(2);
        sOUT.mBattery = x(3);
        sOUT.mMotors = x(4);
        sOUT.mtow = x(5);
        sOUT.Ereserve = x(6);
        sOUT.S=x(7);
        sOUT.rpm=x(8);
        sOUT.eta_motor=x(9);
        sOUT.m_gb=x(10);
        
        
        sOUT.x = x;
        sOUT.xU = [];%xU is empty
        sOUT.xL = xL;
        %sOUT.vU = vU;
        %sOUT.vL = vL;
        %sOUT.wU = wU;
        %sOUT.wL = wL;
        sOUT.misc = misc;
        %sOUT.v = v;
        %sOUT.w = w;
        sOUT.c=c;
        %sOUT.phi=phi;
        sOUT.foriginal=foriginal;
        %sOUT.fATC = fATC;
        
        
    end
%Constraint
    function [cn,ceqn] = mycon_fix(x,a)
    x(1)=a;
    Pwing_output=Pwing(x(5),x(7));%2 outputs (mtow,ss)
    Pmotor_output=Pmotor(x(8),x(9)); % 2 outputs (rpm,eta_motorm)
    Pgearbox_output=Pgearbox(x(1),x(2),x(5),x(7),x(8),x(9),x(10)); % 7 outputs (rest 7 outputs)
    %Pprop_output=Pprop(x(1),x(2),x(3),x(4),x(6),x(10)); % 3 outputs
    %misc.range=rangei;
   
        rangei=50000;
        misc.range=rangei;
        misc.vehicle='tiltwing';
        misc.payload=300;
        rProp = x(1);
        V = x(2);
        mBattery = x(3);
        mMotors = x(4);
        mtow = x(5);
        Ereserve=x(6);
        S=x(7);
        rpm=x(8);
        eta_motor=x(9);
        m_gb=x(10);
        vehicle=misc.vehicle;
        range=misc.range;
        payload=misc.payload;

        ceqn(1) = diffc(Pwing_output(1),mtow);  %from w
        ceqn(2) = diffc(Pwing_output(2),S); %r
        ceqn(3) = diffc(  Pmotor_output(1),rpm);   %w
        ceqn(4) = diffc( Pmotor_output(2),eta_motor);  %w
        ceqn(5) = diffc( Pgearbox_output(1),rProp);  %c
        ceqn(6) = diffc(Pgearbox_output(2),V);    %w     
        ceqn(7) = diffc(Pgearbox_output(3),mtow);  %w
        ceqn(8) = diffc(Pgearbox_output(4),S);  %c
        ceqn(9) = diffc(Pgearbox_output(5),rpm);    %w   
        ceqn(10) = diffc(Pgearbox_output(6),eta_motor);    %w  
        ceqn(11) = diffc(Pgearbox_output(7),m_gb);    %w
        
        % Assumed values
        batteryEnergyDensity = 230; % Expected pack energy density in 3-5 years [Wh/kg]
        motorPowerDensity = 5;
        dischargeDepthReserve = 0.95; % Can only use 95% of battery energy in reserve mission
        
       [~,~,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        
        
        
        % Mass estimate
        mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);
        
        

        

        
        % Constraint on MTOW
        cn(1) = mass.W - mtow * 9.8;
        
        
        
        % Battery sizing
        [EReserve,~,~,~,~] = reserveMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        cn(2) = EReserve - mBattery * batteryEnergyDensity * dischargeDepthReserve / 1000;
        
%         % motor power density
%         cn(4) = hoverOutput.PMax / 1000 - mMotors * motorPowerDensity;
        
        
        %motor sizing based on torque
        torq=0.74*(hoverOutput.PMax/8)/(rpm*2*pi/60);
        lb2kg = 0.453592;   
        cn(3) = 0.3928*(torq^0.8587)*lb2kg*8 - mMotors;
        
        
%         % Constraint on tip speed
%         tip_mach=0.65;
%         omega=tip_mach*340.294/rProp;
%         rpm_max=omega*60/(2*pi);
%         cn(4)=rpm-rpm_max;
%         cn(4)=0;
        

        
        % Ereserve constraint
        ceqn(12)=abs (EReserve-x(6)); 
    end

    function cd= diffc(aT,aR)
       % cd=-(aT-aR)/(aT) ; %target - response
        cd=(aT-aR).^2 ;
%         c=abs(c);
    end



