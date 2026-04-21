function [am, bias_out] = imu_model(a_true, bias_in, dt, params, rng_obj)
bias_out = bias_in + params.bias_instability*sqrt(dt)*rng_obj.randn(3,1);
am = a_true + bias_out + params.accel_noise*rng_obj.randn(3,1);
end
