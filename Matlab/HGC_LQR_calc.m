function K_poly_coeffs = HGC_LQR_calc()
    % 定义符号变量
    syms theta thetad1 thetad2;
    syms x xdot1 xdot2;
    syms phi phidot1 phidot2;
    syms T Tp N P Nm Pm Nf t;
    syms R L Lm l Mw Mp M Iw Ip Im g;
    % 固定参数

    g_val =9.81 ;   % 重力加速度，单位：m/s²
    R_val =0.06 ;    % 驱动轮半径，单位：m
    l_val =0.02 ;   % 机体重心到其转轴距离，单位：m
    Mw_val =0.55 ;   % 驱动轮转子质量，单位：kg
    Mp_val =0.6185 ;     % 摆杆质量，单位：kg
    M_val =20 ;     % 机体质量，单位：kg
    a_body=0.4848;% 机体沿前进方向长度
    b_body=0.3256;% 机体沿高度（含云台）
   

    % LQR权重矩阵    摆角     位移     机体角
   Q_matrix = diag([500, 2, 60, 10, 4000, 1]);
    %               驱动轮  髋关节
    R_matrix = diag([3.0, 0.19]);

    % 1. 定义系统动力学方程
    Nm = M*( xdot2 + (L+Lm)*thetad2*cos(theta) - (L+Lm)*(thetad1)^2*sin(theta)  - l*phidot2*cos(phi) + l*(phidot1)^2*sin(phi) );
    N  = Mp*(xdot2 + L*thetad2*cos(theta) - L*(thetad1)^2*sin(theta)) + Nm;
    Pm  = M*( -(L+Lm)*thetad2*sin(theta) - (L+Lm)*(thetad1)^2*cos(theta) - l*phidot2*sin(phi) - l*(phidot1)^2*cos(phi) ) + M*g;
    P  = Mp*(-L*thetad2*sin(theta) - L*(thetad1)^2*cos(theta)) + Pm + Mp*g;
    
    eq1 = xdot2 - (T - N*R)/(Iw/R +Mw*R);
    eq2 = (P*L + Pm*Lm)*sin(theta) - (N*L + Nm*Lm)*cos(theta) - T + Tp -Ip*thetad2;
    eq3 = Tp + Nm*l*cos(phi) + Pm*l*sin(phi) -Im*phidot2;
    
    [xdot2_c, thetad2_c, phidot2_c] = solve([eq1,eq2,eq3], [xdot2, thetad2, phidot2]);
    
    % 状态变量和输入变量
    X = [theta; thetad1; x; xdot1; phi; phidot1];
    U = [T; Tp];
    Xdot = [thetad1; thetad2_c; xdot1; xdot2_c; phidot1; phidot2_c];
    
    % 计算雅可比矩阵得到A, B
    A = jacobian(Xdot, X);
    B = jacobian(Xdot, U);
    
    % 代入系统平衡点 (线性化)
    A = subs(A, [theta; thetad1; xdot1; phi; phidot1; T; Tp], [0;0;0;0;0;0;0]); 
    B = subs(B, [theta; thetad1; xdot1; phi; phidot1; T; Tp], [0;0;0;0;0;0;0]); 
    
    
    
    %输入腿长范围
    min_leg_length = 0.12;
    max_leg_length = 0.33; 
    leg_lengths = (min_leg_length : 0.01 : max_leg_length)';
    num_points = length(leg_lengths);
    
    %  分配存储K矩阵的数组
    K_all = zeros(2, 6, num_points); 
    
    %  循环计算每个腿长对应的K矩阵

    for i = 1:num_points
        current_L = leg_lengths(i);
        current_Lm = current_L; % 假设L和Lm相等
        
         Iw_val = (1/2)*Mw_val*R_val^2;     % 驱动轮转子转动惯量，单位：kg·m²
         Im_val = (1/12)*M_val*(a_body^2+b_body^2);     % 机体绕质心转动惯量，单位：kg·m²
         Ip_val = (1/12)*Mp_val*(current_L+current_Lm)^2;       % 摆杆绕质心转动惯量，单位：kg·m² 

        % 将当前腿长代入A, B矩阵
        A_current = subs(A, [g, R, L, Lm, l, Mw, Mp, M, Iw, Ip, Im], ...
                           [g_val, R_val, current_L, current_Lm, l_val, Mw_val, Mp_val, M_val, Iw_val, Ip_val, Im_val]);
        B_current = subs(B, [g, R, L, Lm, l, Mw, Mp, M, Iw, Ip, Im], ...
                           [g_val, R_val, current_L, current_Lm, l_val, Mw_val, Mp_val, M_val, Iw_val, Ip_val, Im_val]);
        
        % 将符号矩阵转换为数值矩阵
        A_num = double(vpa(A_current));
        B_num = double(vpa(B_current));
        
        % 使用LQR计算增益矩阵K
        K_lqr = lqr(A_num, B_num, Q_matrix, R_matrix);
        K_all(:, :, i) = K_lqr;
        
        
        if mod(i, 10) == 0
       
        end
    end
 
    
    % 5. 多项式拟合 (三次多项式)
    degree = 3;
    K_poly_coeffs = zeros(2, 6, degree+1); 
    

    for row = 1:2
        for col = 1:6
            % 提取当前元素在所有腿长下的值
            K_element = squeeze(K_all(row, col, :));
            % 进行三次多项式拟合 p = p1*x^3 + p2*x^2 + p3*x + p4
            p = polyfit(leg_lengths, K_element, degree);

            K_poly_coeffs(row, col, :) = p;
        end
    end
    
    fid = fopen('output.txt', 'w');

    if fid == -1
        error('无法创建或打开output.txt文件');
    end
    
fprintf(fid, '{\\\n');
for row = 1:2
    fprintf(fid, '        { ');
    for col = 1:6
        coeffs = squeeze(K_poly_coeffs(row, col, :));

        fprintf(fid, '{%.6ff, %.6ff, %.6ff, %.6ff}', ...
                coeffs(4), coeffs(3), coeffs(2), coeffs(1));

        if col < 6
            fprintf(fid, ', \\\n          ');  
        else
            fprintf(fid, ' }'); 
        end
    end
    

    if row < 2
        fprintf(fid, ', \\\n'); 
    else
        fprintf(fid, ' \\\n');    
    end
end
fprintf(fid, '    }');  


fclose(fid);
output_filename = 'output.txt';
winopen(output_filename);



file_path = fullfile(pwd, 'output.txt');
fprintf('文件保存路径: %s\\n', file_path);
end