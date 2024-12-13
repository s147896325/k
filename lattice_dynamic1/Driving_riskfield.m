% --2023--
% by iL
classdef Driving_riskfield <handle
    properties
        % obstacle and host veh states
        % [X,Y,Vx]
        Obs_states;
        Hv_states;

        % shape coffi
        W_obs=1.0;
        L_obs=4.4;  
        % static field parametes
        kx=4.0;
        ky=1.0;  % lateral direction

        Aob=15;
        belta=2;

        % dynamic field parametes
        k_v=5.0;
        alpha=2.0;%相对速度系数

        % Driving risk field val_matrix
        Sta_Field;
        Dyn_Field;
        Total_fieldVal;
    end
    methods

        % construction
        function Obj=Driving_riskfield()
        end
        % total dynamic and visuallization

        function Total_fieldVal=Total_drf(Obj)
            % calc the total field val
            Obj.Static_field;
            Obj.Dynamic_field;
            Obj.Total_fieldVal=Obj.Sta_Field+Obj.Dyn_Field;
            Total_fieldVal=Obj.Total_fieldVal;
        end

        % static driving risk field
        function Static_field(Obj)
            % shape function of an obstacle veh
            Segma_y=Obj.ky*Obj.W_obs;
            Segma_x=Obj.kx*Obj.L_obs;

            for e=1:1:size(Obj.Obs_states,2)
                tmp=((((Obj.Hv_states(1)-Obj.Obs_states{e}(1))^2)/(Segma_x^2))^Obj.belta +(((Obj.Hv_states(2)-Obj.Obs_states{e}(2))^2)/(Segma_y^2))^Obj.belta);            
                Aob_Static{e}=Obj.Aob*exp(-tmp);
            end

            for i=1:1:size(Obj.Obs_states,2)
                if i==1
                    Obj.Sta_Field=Aob_Static{1};
                else
                    Obj.Sta_Field=Obj.Sta_Field+Aob_Static{i};
                end
            end
        end

        % dynamic driving risk field considering changed vel and relative dist
        function Dynamic_field(Obj)

            Segma_y=Obj.ky*Obj.W_obs;

            for e=1:1:size(Obj.Obs_states,2)
                if Obj.Obs_states{e}(3)>=Obj.Hv_states(3)
                    rel_v=1;
                else
                    rel_v=-1;
                end

                segma_v=Obj.k_v*abs(Obj.Obs_states{e}(3)-Obj.Hv_states(3));

                tmp_D=( ((Obj.Hv_states(1)-Obj.Obs_states{e}(1))^2)/(segma_v^2) +((Obj.Hv_states(2)-Obj.Obs_states{e}(2))^2)/(Segma_y^2));

                Aob_dynamic{e}=Obj.Aob*exp(-tmp_D)/(1+exp(rel_v*(Obj.Hv_states(1)-Obj.Obs_states{e}(1)-Obj.alpha*Obj.L_obs*rel_v)));
            end

      
            for i=1:1:size(Obj.Obs_states,2)
                if i==1
                    Obj.Dyn_Field=Aob_dynamic{i};
                else
                    Obj.Dyn_Field=Obj.Dyn_Field+Aob_dynamic{i};
                end
            end
        end
    end
end