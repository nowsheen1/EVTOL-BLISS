function [xopt]=Pgearbox(rPropg,Vg,mtowg,Sg,rpmg,eta_motorg,m_gbg)
% clear
% close all

disp(' ')
disp('----Gearbox Begins----')


%         AR = x(1);
%         S = x(2); %m2
%         rootaoa = x(3); %degree
%         vinf = x(4); %cruise velocity- m/s
%         rProp = x(5); %rotor radius, m
lb=[0.01,10,100,1,2600,0.01,20];
ub=[10, 100, 9999,30,8800,1,200];
A =[];
b = [];
Aeq = [];
beq = [];


constraints=@(x)  mycon;
func=@(x) myfun;

%options = optimoptions('surrogateopt','Display','iter','MaxFunctionEvaluations',9000,'Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr');% ,'PlotFcn','optimplotfvalconstr');
options=optimoptions('surrogateopt','Display','iter','MaxFunctionEvaluations',900);
%Run optimization for PS
[xopt,fvalopt, FLAG, OUTPUT] = surrogateopt(@(x) myfun(x,rPropg,Vg,mtowg,Sg,rpmg,eta_motorg,m_gbg),lb,ub,options);

%[~,sOUT]=myfun1(xopt,xU,xLf,vU,vLf,wU,wL,misc);
%sOUT.xopt=xopt;
%sOUT.funcCount = OUTPUT.funccount;
disp('______Wing ends_________')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Objective
    function [foriginal,ceqn]=myfun(x,rPropg,Vg,mtowg,Sg,rpmg,eta_motorg,m_gbg)
        %do calculations for both objective and constraints here itself.
        %save and load in constraint section
         rangei=50000;
         misc.range=rangei;
         misc.vehicle='tiltwing';
         misc.payload=300;
        rProp=x(1);
        V=x(2);
        mtow=x(3);
        S=x(4);
        rpm=x(5);
        eta_motor=x(6);
        m_gb=x(7);
        
        %get original objective
         %foriginal =m_gb ;
        
        %Assemble c: target - response
        
        c(1) = diffc(rPropg,rProp);
        c(2) = diffc(Vg,V);
        c(3) = diffc(mtowg,mtow);
        c(4) = diffc(Sg,S) ;
        c(5) = diffc(rpmg,rpm);
        c(6) = diffc(eta_motorg,eta_motor);
        c(7) = diffc(m_gbg,m_gb);%Assemble v and w.Make it consistently ordered
        foriginal=(1/7)*sum(c);
        %get all phi
        %v = vU; w=wU;
        %phi= v.*c+(w.*c).^2;
        
        %ATC objective
        %fATC=foriginal+ sum(phi);
        
        %Pack all data to a struct
        sOUT.rProp=x(1);
        sOUT.V=x(2);
        sOUT.mtow=x(3);
        sOUT.S=x(4);
        sOUT.rpm=x(5);
        sOUT.eta_motor=x(6);
        sOUT.m_gb=x(7);
        sOUT.x = x;
        sOUT.xU = [rPropg,Vg,mtowg,Sg,rpmg,eta_motorg,m_gbg];
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
        sOUT.foriginal=foriginal;
        %sOUT.fATC = fATC;
        rProp=x(1);
        V=x(2);
        mtow=x(3);
        S=x(4);
        rpm=x(5);
        eta_motor=x(6);
        m_gb=x(7);
        
        
        vehicle=misc.vehicle;
        range=misc.range;
        
        
        
        [~,~,hoverOutput,~] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        %ceqn(1)= x(7)-mass_gb(rpm,rProp,hoverOutput.PMax);
        %ceqn(2)= -(x(7)-mass_gb(rpm,rProp,hoverOutput.PMax));
        ceqn(1)= (x(7)/mass_gb(rpm,rProp,hoverOutput.PMax))-1;
        ceqn(2)= (x(7)/mass_gb(rpm,rProp,hoverOutput.PMax))+1;
        cn=[];
 
    end

 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Constraint
   % function [cn,ceqn] = mycon(x,xU,xL, vU,vL,wU,wL,misc)
        
        
    %    rProp=x(1);
     %   V=x(2);
      %  mtow=x(3);
       % S=x(4);
        %rpm=x(5);
        %eta_motor=x(6);
        %m_gb=x(7);
        
        
        %vehicle=misc.vehicle;
        %range=misc.range;
        
        
        
        %[~,~,hoverOutput,~] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        %ceqn= x(7)-mass_gb(rpm,rProp,hoverOutput.PMax);
        %cn=[];
    %end


    function cd= diffc(aT,aR)
        %cd=0.5*(aT-aR).^2; %target - response
           cd=(aT-aR).^2;
        %                c=abs(c);
    end

end


