function [xopt]=Pmotor(rpms,eta_motors)
% clear
% close all

disp(' ')
disp('----Motor Begins----')


%         AR = x(1);
%         S = x(2); %m2
%         rootaoa = x(3); %degree
%         vinf = x(4); %cruise velocity- m/s
%         rProp = x(5); %rotor radius, m
lb=[2600,0.01];
ub= [8800,1];
A =[];
b = [];
Aeq = [];
beq = [];


constraints=@(x)  mycon;
func=@(x) myfun;

%options = optimoptions('surrogateopt','Display','iter','MaxFunctionEvaluations',900,'Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr');% ,'PlotFcn','optimplotfvalconstr');
options=optimoptions('surrogateopt','Display','iter','MaxFunctionEvaluations',900);
%Run optimization for PS
[xopt,fvalopt, FLAG, OUTPUT] = surrogateopt(@(x) myfun(x,rpms,eta_motors),lb,ub,options);

%[~,sOUT]=myfun1(xopt,xU,xLf,vU,vLf,wU,wL,misc);
%sOUT.xopt=xopt;
%sOUT.funcCount = OUTPUT.funccount;
disp('______Wing ends_________')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Objective
    function [foriginal,ceqn]=myfun(x,rpms,eta_motors)
        %do calculations for both objective and constraints here itself.
        %save and load in constraint section
         rangei=50000;
         misc.range=rangei;
         misc.vehicle='tiltwing';
         misc.payload=300;
        rpm = x(1);
        eta_motor = x(2); %m2
        
        
        
        
        %get original objective
        %foriginal =eta_motor ;
        
        
        %Assemble c: target - response
        
        c(1) = diffc(rpms,rpm);
        c(2) = diffc(eta_motors,eta_motor);
        foriginal=0.5*sum(c);
        %Assemble v and w.Make it consistently ordered
        %get all phi
        %v = vU; w=wU;
        %phi= v.*c+(w.*c).^2;
        
        %ATC objective
        %fATC=foriginal+ sum(phi);
        
        sOUT.rpm=rpm;
        sOUT.eta_motor=eta_motor;
        sOUT.x = x;
        sOUT.xU = [rpms,eta_motors];
        sOUT.xL = [];
        %sOUT.vU = vU;
        %sOUT.vL = vL;
        %sOUT.wU = wU;
        %sOUT.wL = wL;
        sOUT.misc = misc;
        %sOUT.v = v;
        %sOUT.w = w;
        sOUT.c=c;
        %sOUT.phi=phi;
        %sOUT.foriginal=foriginal;
        %sOUT.fATC = fATC;
         rpm=x(1);
        %ceqn(1)= x(2)-motor_eta(rpm);
        %ceqn(2)= -(x(2)-motor_eta(rpm));
        ceqn(1)= (x(2)/motor_eta(rpm))-1;
        ceqn(2)= (x(2)/motor_eta(rpm))+1;
        cn=[];
        
        
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Constraint
   % function [cn,ceqn] = mycon(x,xU,xL, vU,vL,wU,wL,misc)
        
    %    rpm=x(1);
     %   ceqn(1)= x(2)-motor_eta(rpm);
      %  cn=[];
        
    %end


    function cd= diffc(aT,aR)
        %cd=0.5*(aT-aR).^2 ; %target - response
           cd=(aT-aR).^2 ;
        %                c=abs(c);
    end

end


