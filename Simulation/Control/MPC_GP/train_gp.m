clear
clc
close all

%%%%%%%%%%%%%
%% to edit %%
%%%%%%%%%%%%%
%enter filename of data in format previously send via email
%output will be the same file name with prefix "trained_" to denote the
%trained hyperparameters used in write_matrices_trajectory.m files
% data_list = [5000,2000,1000,500,200,100,50,20,10,5,2];
data_list = [5];
v_list = [1,2,3,4,5];
% filename = strcat('new_inputs_outputs_with_05_data_only_',num2str(data_list(counter)),'.mat')
%-------------

%loop trough filenames (for different versions)
for counter=1:length(v_list)
    filename = strcat('new_inputs_outputs_5_data_v',num2str(v_list(counter)),'.mat')
    load(filename);
    x = input_training(:,:);
    y = output_training(:,:);
    frac_train = 1;

    %% Split data into training/testing
    fun_total = [];
    for lv1=1:size(y, 2)
        input_training = x(1:floor(frac_train*size(x,1)),:);
        output_training = y(1:floor(frac_train*size(y,1)),lv1);
        input_test = x(floor(frac_train*length(x))+1:end,:);
        output_test = y(floor(frac_train*length(x))+1:end,lv1);

        %%
        % x = input_training;
        % real_y = (sum(input_training,2));
        % y_training = (sum(input_training,2))+normrnd(0,1, length(input_training),1).*abs(cos((x(:,1).^2+x(:,2).^2)/2));
        % input_test = input_training;
        % y_test = y_training;
        % [X1, X2] = meshgrid(x1, -1:0.05:1); 
        % x_grid = [X1(:), X2(:)];


        %% Learning and Predicting grid
        %[~, ~, Ey_test, Vy_test,  mutst, diagSigmatst, atst, diagCtst, alpha, covfunc1, theta1, X, lengthscales] = vhgpr_ui(input_training, y_training, x_grid, X1(:)*0);
        a = tic;
        [~, ~, Ey_test{lv1}, Vy_test{lv1},  mutst{lv1}, diagSigmatst{lv1}, atst{lv1}, diagCtst{lv1}, alpha{lv1}, covfunc1{lv1}, theta1{lv1}, X{lv1}, lengthscales{lv1}] = vhgpr_ui(input_training, output_training, input_training, output_training);
        toc
        %{
        %% Use interpolation for new predictions
        vhgp_mean_eval = fit(x_grid, Ey_test,'linearinterp');
        vhgp_std_eval = fit(x_grid, sqrt(Vy_test),'linearinterp');
        %% Predict test
        predicted_mean = vhgp_mean_eval(input_test);
        predicted_std = vhgp_std_eval(input_test);

        %% Plotting results
        std_y_test = sqrt(Vy_test);
        % figure; plot3(input_test(:,1), input_test(:,2),predicted_mean); hold on;  
        % plot3(input_test(:,1), input_test(:,2),real_y); plot3(input_test(:,1), input_test(:,2),y_test, '.');  
        % plot3(input_test(:,1), input_test(:,2),predicted_mean+predicted_std, 'k'); 
        % plot3(input_test(:,1), input_test(:,2),predicted_mean-predicted_std, 'k');
        %}
        %% Compute mean using GP formula
        % x_grid = x_grid./(ones( size(x_grid,1) ,1)*exp(lengthscales(:)'));
        % [~, K1star] = feval(covfunc1{:}, theta1, X, x_grid);
        % new_Ey_test = K1star'*alpha;

        u = sym('u', [size(input_training,2),1]);
        k_star{lv1} = [];
        for i=1:length(input_training)
            u_tmp{lv1} = u./exp(lengthscales{lv1}(:));
            [k, dk] = covFunc_data(theta1{lv1}, X{lv1}(i,:)', u_tmp{lv1}, lengthscales{lv1});
            k_star{lv1} = [k_star{lv1}, k];
        end
        fun{lv1} = k_star{lv1}*alpha{lv1};
        fun_total = [fun_total;fun{lv1}];
    %     f_gp = matlabFunction(k_star{lv1}*alpha{lv1}, 'Vars', {xp}, 'File', 'twist_b_gp');
    end

    data.Ey_test=Ey_test;
    data.Vy_test=Vy_test;
    data.mutst=mutst;
    data.diagSigmatst=diagSigmatst;
    data.atst=atst;
    data.diagCtst=diagCtst;
    data.alpha=alpha;
    data.covfunc1=covfunc1;
    data.theta1=theta1;
    data.X=X;
    data.lengthscales=lengthscales;

    save_file = strcat('trained_', filename);
    save(save_file, 'data');
end

% save('learning_output_03_19_2018', 'data');
% twist_b1 = matlabFunction(fun{1}, 'Vars', {u}, 'File', 'twist_b_gp1_data');
% twist_b2 = matlabFunction(fun{2}, 'Vars', {u}, 'File', 'twist_b_gp2_data');
% twist_b3 = matlabFunction(fun{3}, 'Vars', {u}, 'File', 'twist_b_gp3_data');

% f = k_star*alpha;
% 
% 
% for i=1:length(input_test)
%     input(i,:) = input_test(i,:);
%     input(i,:) = input(i,:)./(ones( size(input(i,:),1) ,1)*exp(lengthscales(:)'));
% 
%     output(i,:) = f_gp(input(i,:)');
% end
% 
% % A = jacobian(k_star*alpha, xp);
% figure; 
% %plot3(input_test(:,1), input_test(:,2),predicted_mean); hold on;  
% plot3(input_test(:,1), input_test(:,2),output, 'm'); hold on;
% plot3(input_test(:,1), input_test(:,2),real_y); hold on;
% plot3(input_test(:,1), input_test(:,2),y_test, '.');  hold on;
% %plot3(input_test(:,1), input_test(:,2),predicted_mean+predicted_std, 'k'); hold on;
% %plot3(input_test(:,1), input_test(:,2),predicted_mean-predicted_std, 'k');