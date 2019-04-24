function [uncertain_ss] = get_uncertain_ss(input_ss)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
nominal_A = input_ss.A;
nominal_B = input_ss.B;
nominal_C = input_ss.C;
nominal_D = input_ss.D;
A_array = umat(zeros(size(nominal_A)));
B_array = umat(zeros(size(nominal_B)));
C_array = umat(zeros(size(nominal_C)));
D_array = umat(zeros(size(nominal_D)));
ORDER = size(nominal_A, 1);
INPUT_DIMENSION = size(nominal_B, 2);
OUTPUT_DIMENSION = size(nominal_C, 1);
for i=1:ORDER
    for j = 1:ORDER
        if i==j
            A_array(i,j) = nominal_A(i,j);
        else
            if abs(nominal_A(i,j)) == 0
                A_array(i, j) = ureal("A" + string(i) + string(j), nominal_A(i,j), 'Range', [-1e-5, 1e-5]);
            else
                A_array(i, j) = ureal("A" + string(i) + string(j), nominal_A(i,j), 'Percentage', 30);
            end
        end
    end
end

for i=1:ORDER
    for j=1:INPUT_DIMENSION
        if abs(nominal_B(i,j)) == 0
            B_array(i, j) = ureal("B" + string(i) + string(j), nominal_B(i,j), 'Range', [-1e-5, 1e-5]);
        else
            B_array(i, j) = ureal("B" + string(i) + string(j), nominal_B(i,j), 'Percentage', 30);
        end
    end
end

for i = 1:OUTPUT_DIMENSION
%     for j = 1:ORDER
%         if abs(nominal_C(i,j)) < 0.003
%             C_array(i, j) = ureal("C" + string(i) + string(j), nominal_C(i,j), 'Range', [-0.003, 0.003]);
%         else
%             C_array(i, j) = ureal("C" + string(i) + string(j), nominal_C(i,j), 'Percentage', 30);
%         end
%         
%     end
    for j = 1:ORDER
        C_array(i,j) = nominal_C(i,j);
    end

    for j = 1:INPUT_DIMENSION
        D_array(i, j) = nominal_D(i,j);
    end
end

uncertain_ss = ss(A_array, B_array, C_array, D_array, input_ss.Ts);
end

