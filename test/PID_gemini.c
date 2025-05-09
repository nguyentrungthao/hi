#include <stdio.h>
#include <math.h> // Để dùng fabsf

// Cấu trúc lưu trữ các thông số và trạng thái của bộ điều khiển PID
typedef struct {
    // Tham số PID do người dùng cung cấp
    float Kp; // Độ lợi tỉ lệ
    float Ki; // Độ lợi tích phân (Ki = Kp_sach / Ti_sach)
    float Kd; // Độ lợi vi phân (Kd = Kp_sach * Td_sach)
    float Kw; // Độ lợi anti-windup (Kw = 1/Tt_sach)

    // Tham số bộ lọc vi phân
    float N_filter; // Hệ số lọc vi phân (thường từ 8-20)

    // Thời gian lấy mẫu
    float dt; // Thời gian lấy mẫu (s)

    // Giới hạn đầu ra
    float out_min; // Giới hạn dưới của đầu ra
    float out_max; // Giới hạn trên của đầu ra

    // Trạng thái nội bộ
    float integral_term_state;     // Trạng thái của thành phần Tích phân (I-term output)
    float prev_pv;                 // Giá trị biến quá trình ở bước trước
    float prev_derivative_term;    // Giá trị thành phần Vi phân (D-term output) ở bước trước

    float prev_unclamped_output;   // Tín hiệu điều khiển chưa bão hòa ở bước trước (cho anti-windup)
    float prev_clamped_output;     // Tín hiệu điều khiển đã bão hòa ở bước trước (cho anti-windup)

} PID_Thermal_Controller;

// Hàm khởi tạo bộ điều khiển PID
void pid_thermal_init(PID_Thermal_Controller* pid,
    float Kp, float Ki, float Kd, float Kw, float N_filter,
    float dt, float out_min, float out_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kw = Kw;
    pid->N_filter = (N_filter < 1.0f) ? 1.0f : N_filter; // Đảm bảo N >= 1

    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->integral_term_state = 0.0f;
    pid->prev_pv = 0.0f; // Nên khởi tạo bằng giá trị pv thực tế ban đầu
    pid->prev_derivative_term = 0.0f;

    pid->prev_unclamped_output = 0.0f;
    pid->prev_clamped_output = 0.0f;
}

// Hàm tính toán đầu ra PID
float pid_thermal_compute(PID_Thermal_Controller* pid, float setpoint, float pv) {
    float error;
    float p_term, i_term_current_contribution, d_term;
    float unclamped_output;
    float clamped_output;

    // 1. Tính sai số
    error = setpoint - pv;

    // 2. Tính thành phần Tỉ lệ (Proportional)
    p_term = pid->Kp * error;

    // 3. Tính thành phần Vi phân (Derivative) - Tác động lên PV, có lọc
    // Td_sach = Kd / Kp (nếu Kp != 0)
    // Tf_derivative = Td_sach / N_filter = (Kd / Kp) / N_filter
    float Td_book = 0.0f;
    if (fabsf(pid->Kp) > 1e-6f) {
        Td_book = pid->Kd / pid->Kp;
    }

    float Tf_derivative;
    if (pid->N_filter > 0.0f && Td_book > 0.0f) {
        Tf_derivative = Td_book / pid->N_filter;
    }
    else {
        // Nếu N hoặc Td_book không hợp lệ, dùng giá trị nhỏ để lọc nhanh hoặc không lọc
        // Hoặc có thể coi D-term = 0 nếu Td_book = 0
        Tf_derivative = pid->dt / 2.0f; // Giá trị nhỏ để ít lọc nhất nếu Td_book > 0
        if (Td_book <= 1e-6f) { // Nếu Kd hoặc Kp là 0, Td_book sẽ là 0
            Tf_derivative = 1.0f; // giá trị bất kỳ > dt để không gây lỗi chia cho 0 bên dưới
        }
    }

    if (Td_book > 1e-6f && pid->dt > 0.0f) { // Chỉ tính D-term nếu Kd (và Kp) có nghĩa
        // d_term = (Tf_d / (Tf_d + dt)) * prev_d_term + (Kd_user / (Tf_d + dt)) * (prev_pv - pv)
        d_term = (Tf_derivative / (Tf_derivative + pid->dt)) * pid->prev_derivative_term +
            (pid->Kd / (Tf_derivative + pid->dt)) * (pid->prev_pv - pv);
    }
    else {
        d_term = 0.0f;
    }
    pid->prev_derivative_term = d_term; // Lưu lại D-term cho lần tính sau của bộ lọc


    // 4. Tính thành phần Tích phân (Integral) với Anti-Windup (Back-Calculation theo Hình 3.3)
    // integral_term_state là đầu ra của khối 1/s (sau khi đã cộng dồn)
    // d(integral_term_state)/dt = Ki_sach * error + (1/Tt_sach) * (prev_clamped_output - prev_unclamped_output)
    // Ki_sach = Ki_user
    // 1/Tt_sach = Kw_user
    float integrator_input_signal = pid->Ki * error; // Phần từ sai số

    // Thêm phần phản hồi từ back-calculation của bước trước
    integrator_input_signal += pid->Kw * (pid->prev_clamped_output - pid->prev_unclamped_output);

    pid->integral_term_state += integrator_input_signal * pid->dt;

    // Giới hạn giá trị tích lũy của I-term (một dạng anti-windup bổ sung đơn giản, tùy chọn)
    // Ví dụ: giới hạn để I-term không vượt quá toàn bộ dải out_max - out_min
    // float i_term_max = (pid->out_max - pid->out_min);
    // float i_term_min = -(pid->out_max - pid->out_min);
    // if (pid->integral_term_state > i_term_max) pid->integral_term_state = i_term_max;
    // if (pid->integral_term_state < i_term_min) pid->integral_term_state = i_term_min;


    // 5. Tổng hợp tín hiệu điều khiển (chưa bão hòa)
    unclamped_output = p_term + pid->integral_term_state + d_term;

    // 6. Giới hạn đầu ra (Clamping)
    clamped_output = unclamped_output;
    if (clamped_output > pid->out_max) {
        clamped_output = pid->out_max;
    }
    if (clamped_output < pid->out_min) {
        clamped_output = pid->out_min;
    }

    // 7. Cập nhật các giá trị trước đó cho vòng lặp kế tiếp
    pid->prev_pv = pv;
    pid->prev_unclamped_output = unclamped_output;
    pid->prev_clamped_output = clamped_output;

    return clamped_output;
}


// --- Ví dụ sử dụng ---
int main() {
    PID_Thermal_Controller oven_pid;
    float Kp_val = 2.5f;
    float Ki_val = 0.02f; // Ki = Kp_sach/Ti_sach. Nếu Ti_sach=125, Kp_sach=2.5 => Ki=0.02
    float Kd_val = 15.0f;  // Kd = Kp_sach*Td_sach. Nếu Td_sach=6, Kp_sach=2.5 => Kd=15
    float Kw_val = 0.1f;   // Kw = 1/Tt_sach. Nếu Tt_sach=Ti_sach=125 => Kw = 1/125 = 0.008. Hoặc Kw là độ lợi riêng.
    // Trong Hình 3.3, 1/Tt là độ lợi của nhánh phản hồi.
    // Nếu Tt = Ti_sach, thì Kw = Ki_val / Kp_val = 0.02/2.5 = 0.008
    // Nếu Tt = Kp_sach (theo conditioning technique), thì Kw = 1/Kp_val = 1/2.5