function [xopt]=Pwing(mtows,Ss)
%[rProp,V,mBattery,MMotor,
%mtow,Ereserve,S, rpm, eta_motor,m_gb]
% clear
% close all

disp(' ')
disp('----Wing Begins----')


%         AR = x(1);
%         S = x(2); %m2
%         rootaoa = x(3); %degree
%         vinf = x(4); %cruise velocity- m/s
%         rProp = x(5); %rotor radius, m
lb=[100,1];
ub= [9999,30];
A =[];
b = [];
Aeq = [];
beq = [];


constraints=@(x)  mycon;
func=@(x) myfun;

options = optimoptions('surrogateopt','Display','iter','MaxFunctionEvaluations',900);% ,'PlotFcn','optimplotfvalconstr');
%Run optimization for PS
[xopt,fvalopt, FLAG, OUTPUT] = surrogateopt(@(x)myfun(x,mtows,Ss),lb,ub,options);

%[~,sOUT]=myfun(xopt);
%sOUT.xopt=xopt;
%sOUT.funcCount = OUTPUT.funccount;
disp('______Wing ends_________')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Objective
    %function [fATC,sOUT]=myfun(x,xU,xL, vU,vL,wU,wL,misc)
    function [foriginal,ceqn]=myfun(x,mtows,Ss) %changed this line
        %do calculations for both objective and constraints here itself.
        %save and load in constraint section
        rangei=50000;
        misc.range=rangei;
        misc.vehicle='tiltwing';
        misc.payload=300;
        mtow = x(1);
        S = x(2); %m2
        
        
        %get original objective
        %foriginal =mtow ;
        
        
        %Assemble c: target - response
        
        c(1) = diffc(mtows,mtow);
        c(2) = diffc(Ss,S);
        foriginal=0.5*sum(c);
        
        %Assemble v and w.Make it consistently ordered
        %get all phi
        %v = vU; w=wU;
        %phi= v.*c+(w.*c).^2;
        
        %ATC objective
        %fATC=foriginal+ sum(phi);
        %fATC=foriginal; % changed this line
        
        sOUT.mtow=mtow;
        sOUT.S=S;
        
        sOUT.x = x;
        sOUT.xU = [mtows,Ss];
        sOUT.xL = [];
        %sOUT.vU = vU;
        %sOUT.vL = vL;
        %sOUT.wU = wU;
        %sOUT.wL = wL;
        %sOUT.misc = misc;
        %sOUT.v = v;
        %sOUT.w = w;
        sOUT.c=c;
        %sOUT.phi=phi;
        %sOUT.foriginal=foriginal;
        %sOUT.fATC = fATC;
        rho = 1.225;
        % Specify stall conditions
        VStall = 35; % m/s
        CLmax = 1.1; % Whole aircraft CL, section Clmax much higher
        ceqn(1)=x(2)-((x(1)*9.8) / (0.5 * rho * VStall^2 * CLmax));
        ceqn(2)=-(x(2)-((x(1)*9.8) / (0.5 * rho * VStall^2 * CLmax)));
        
        
        
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Constraint
    %function [cn,ceqn] = mycon(x,xU,xL, vU,vL,wU,wL,misc)
        
      %  cn = [];
       % rho = 1.225;
        % Specify stall conditions
        %VStall = 35; % m/s
        %CLmax = 1.1; % Whole aircraft CL, section Clmax much higher
        %ceqn(1)=x(2)-((x(1)*9.8) / (0.5 * rho * VStall^2 * CLmax));
        %ceqn(2)=-(x(2)-((x(1)*9.8) / (0.5 * rho * VStall^2 * CLmax)));
        
   % end


    function cd= diffc(aT,aR)
       %cd=(aT-aR)/(aT) ; %target - response
       cd=(aT-aR).^2 ; %target - response
       %cd=0.5*(aT-aR).^2; %changed this line
       %c=abs(c);
    end
end


