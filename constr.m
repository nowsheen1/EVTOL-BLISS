function [cn,ceqn]=constr(x,vehicle,range,payload) 
zz=x
    %Pprop_output=Pprop(x(1),x(2),x(3),x(4),x(6),x(10)); % 3 outputs
    %misc.range=rangei;
  Pout=Pgearbox(x(5),x(7),x(1),x(2),x(3),x(4),x(5))
        rProp = x(1);
        V = x(2);
        mBattery = x(3);
        mMotors = x(4);
        mtow = x(5);
        S=x(7);
        rpm=x(8);
        eta_motor=x(9);
        m_gb=x(10);
        
        
        
        
        
        
       
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
      
      
        %Assemble v and w.Make it consistently ordered
        %get all phi
        %v = vL; w=wL;
        %phi= v.*c+(w.*c).^2;
        
        %ATC objective
        %fATC=foriginal/1e6+ sum(phi);
        
        %Pack all data to a struct
	    
      
        
        sOUT.x = x;
        sOUT.xU = [];%xU is empty
     
        %sOUT.vU = vU;
        %sOUT.vL = vL;
        %sOUT.wU = wU;
        %sOUT.wL = wL;
        %sOUT.misc = misc;
        %sOUT.v = v;
        %sOUT.w = w;
     
        
  
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