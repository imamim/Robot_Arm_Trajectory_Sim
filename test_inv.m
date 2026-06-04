addpath('lib/dynamics');
J = [1 0; 0 1e-6]; % Nearly singular
disp('Original J:');
disp(J);

disp('Testing get_damped_inverse:');
try
    inv1 = get_damped_inverse(J, 0.1, 0.05);
    disp('get_damped_inverse worked. Result:');
    disp(inv1);
catch ME
    disp('get_damped_inverse failed:');
    disp(ME.message);
end

disp('Testing safe_jacobian_inverse:');
try
    inv2 = safe_jacobian_inverse(J, 0.1, 0.05);
    disp('safe_jacobian_inverse worked. Result:');
    disp(inv2);
catch ME
    disp('safe_jacobian_inverse failed:');
    disp(ME.message);
end
