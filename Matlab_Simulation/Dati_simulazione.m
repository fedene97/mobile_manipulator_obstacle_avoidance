function [v_1,v_2,Oi,Of,dOi,X_i,X_f]=Dati_simulazione(esempio)

global K_path T v0_rep k_e_tra k_e_rot k_e_rotz K1 K2 K3
    % K_path = 10^(-10); % Path curvature parameter 5 good, 10^(-10) if it should stay still
    v_1=0.1;
    v_2=0.5;
switch esempio
    case 1 % Ostacolo movimento alto
        X_i=[0.5  0 1.1 pi pi 0]';
        X_f=[0.5001 0 1.1 pi pi 0]';
        Oi=[ 0.55  6; 
            -0.30  6;
             1.051 6];
        Of=[ 0.55 6;
             0.30 6;
             1.05 6];
        dOi=[0 0; 0 0; 0 0];
        v0_rep=1; 
        k_e_tra=1; 
        k_e_rot=2; 
        k_e_rotz=2; 
        T=5;
        K_path = 10^(-10); 
        K1=2; 
        K2=2; 
        K3=0.5; 
    case 2 % Ostacolo movimento basso
        X_i=[0.5    0 1.1 pi pi 0]';
        X_f=[0.5001 0 1.1 pi pi 0]';
        Oi=[ 0.40  6; 
            -0.50  6;
             0.11  6];
        Of=[ 0.40 6;
             0.50 6;
             0.10 6];
        dOi=[0 0; 0 0; 0 0];
        v0_rep=1; 
        k_e_tra=50; 
        k_e_rot=50; 
        k_e_rotz=50; 
        T=5;
        K_path = 10^(-10); 
        K1=2; 
        K2=2; 
        K3=0.5; 
    case 3 % Ostacolo zone Articolo
        X_i=[0.5  0 1.1 pi pi 0]';
        X_f=[5.0  0 1.1 pi pi 0]';
        Oi=[4     1.00  3; 
            0.05 -0.30  -1.0;
            1.15  0.11  1.10];
        Of=[4     1.00  3;
            0.05 -0.30  1.0;
            1.15  0.10  1.1001];
        dOi=[0 0 0; 0 0 0; 0 0 0];
        v0_rep=1; 
        k_e_tra=1; 
        k_e_rot=2; 
        k_e_rotz=2; 
        T=30;
        K_path = 5; 
        K1=2; 
        K2=2; 
        K3=0.5; 
end

end