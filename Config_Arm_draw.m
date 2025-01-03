function [SM_ele,SM_sec] = Config_Arm_draw(Deform_arm,L0_seg,N_ele,Mode)
% Draw the configuration of the manipulator:
%%
% Deform_arm=[pi/4 pi/4 0, pi/4 -pi/4 0]
% L0_seg=200
% N_ele=10

Num_seg=size(Deform_arm,2)/3;
SM_sec(:,:,1)=eye(4);
Color_seg=linspecer(Num_seg);
L_cord=20;
for ii=1:Num_seg
    Deform_seg=Deform_arm(1,3*(ii-1)+(1:3));
    Bend_XY=Deform_seg(1:2);
    D_L=Deform_seg(3);
    L_seg=L0_seg-D_L;
    Ang_Bend=sqrt(sum(Bend_XY.^2));
    Phi_arc=atan2(Bend_XY(2),Bend_XY(1));
    R_arc=L_seg/Ang_Bend;
    
    if Ang_Bend<1e-5
        for jj=0:N_ele
            SM_ele(:,:,jj+1,ii)=SM_sec(:,:,ii);
            SM_ele(1:3,4,jj+1,ii)=[0, 0, jj*L_seg/N_ele];
        end
    else
        for jj=0:N_ele            
            Ang_Bend_ele=jj*Ang_Bend/N_ele;
            TR_M_ele=axang2rotm([-sin(Phi_arc),cos(Phi_arc),0,Ang_Bend_ele]);
            TP_ele=[cos(Phi_arc)*R_arc*(1-cos(Ang_Bend_ele)), sin(Phi_arc)*R_arc*(1-cos(Ang_Bend_ele)),R_arc*sin(Ang_Bend_ele)]';
            TM_ele=[TR_M_ele,TP_ele;0 0 0 1];
            SM_ele(:,:,jj+1,ii)=SM_sec(:,:,ii)*TM_ele;
        end
    end
    SM_sec(:,:,ii+1)=SM_ele(:,:,end,ii);
    Arc_seg(:,:,ii)=reshape(SM_ele(1:3,4,:,ii),3,[]);
    if ii>1
        hold on
    end
    plot3(Arc_seg(1,:,ii),Arc_seg(2,:,ii),Arc_seg(3,:,ii),'-',color=Color_seg(ii,:),LineWidth=1.5)    
    hold on
    if Mode>=1
    plot3(Arc_seg(1,:,ii),Arc_seg(2,:,ii),0*Arc_seg(3,:,ii),':',color=Color_seg(ii,:),LineWidth=2)
    end
    plot3([SM_sec(1,4,ii+1),SM_sec(1,4,ii+1)+L_cord*SM_sec(1,1,ii+1)],[SM_sec(2,4,ii+1),SM_sec(2,4,ii+1)+L_cord*SM_sec(2,1,ii+1)],[SM_sec(3,4,ii+1),SM_sec(3,4,ii+1)+L_cord*SM_sec(3,1,ii+1)],'r-')
    plot3([SM_sec(1,4,ii+1),SM_sec(1,4,ii+1)+L_cord*SM_sec(1,2,ii+1)],[SM_sec(2,4,ii+1),SM_sec(2,4,ii+1)+L_cord*SM_sec(2,2,ii+1)],[SM_sec(3,4,ii+1),SM_sec(3,4,ii+1)+L_cord*SM_sec(3,2,ii+1)],'g-')
    hold off
     
end

 
end