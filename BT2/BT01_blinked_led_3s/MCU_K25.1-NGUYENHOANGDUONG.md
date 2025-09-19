# So sánh kiến trúc CISC và RISC

---

## 1.1. Giới thiệu khái niệm cơ bản

- **CISC (Complex Instruction Set Computer)**  
  Là kiến trúc máy tính với **tập lệnh lớn và phức tạp**, trong đó một lệnh có thể thực hiện nhiều thao tác (tải dữ liệu, tính toán, lưu kết quả).  
  → Mục tiêu: giảm số lượng lệnh trong chương trình và giúp mã ngắn gọn hơn, nhưng thời gian thực thi có thể chậm do xử lý phức tạp.  

- **RISC (Reduced Instruction Set Computer)**  
  Là kiến trúc máy tính với **tập lệnh đơn giản và nhỏ gọn**, mỗi lệnh chỉ thực hiện **một thao tác cơ bản**.  
  → Mục tiêu: tăng tốc độ và hiệu quả. Bộ xử lý RISC thường sử dụng **pipeline** (chia lệnh thành nhiều giai đoạn, thực hiện đồng thời) để tăng hiệu suất.  

---

## 1.2. Ưu điểm và nhược điểm

### CISC
- **Ưu điểm:**
  - Kích thước mã giảm  
  - Tiết kiệm bộ nhớ  
  - Có khả năng thực hiện lệnh phức tạp  

- **Nhược điểm:**
  - Thực thi chậm hơn  
  - Thiết kế phức tạp  
  - Tiêu thụ điện năng cao  

### RISC
- **Ưu điểm:**
  - Tập lệnh đơn giản  
  - Hỗ trợ pipeline hiệu quả  
  - Tiêu thụ điện năng thấp  

- **Nhược điểm:**
  - Chương trình dài hơn  
  - Khả năng thực hiện lệnh phức tạp bị hạn chế  

---

## 1.3. So sánh theo tiêu chí

### 1.3.1. Cấu trúc tập lệnh
- **CISC:**  
  - Tập lệnh phức tạp, nhiều lệnh  
  - Một lệnh có thể thực hiện nhiều thao tác (load, tính toán, store)  
  - Độ dài lệnh thường **không cố định**  

- **RISC:**  
  - Tập lệnh đơn giản, ít lệnh  
  - Mỗi lệnh thực hiện một thao tác cơ bản  
  - Độ dài lệnh thường **cố định**  

---

### 1.3.2. Tốc độ xử lý
- **CISC:** Một lệnh có thể thực hiện nhiều việc, nhưng thường mất nhiều chu kỳ clock.  
- **RISC:** Mỗi lệnh đơn giản, thường chỉ mất **1 chu kỳ clock** để thực thi.  

---

### 1.3.3. Kích thước chương trình
- **CISC:** Do có lệnh phức tạp → chương trình cần **ít lệnh hơn** → kích thước nhỏ hơn.  
- **RISC:** Do lệnh đơn giản → cần **nhiều lệnh hơn** để làm cùng công việc → kích thước lớn hơn.  

---

### 1.3.4. Độ phức tạp phần cứng
- **CISC:**  
  - Phần cứng phức tạp  
  - Tốn nhiều transistor  
  - Khó tối ưu pipeline  

- **RISC:**  
  - Phần cứng đơn giản  
  - Dễ tối ưu pipeline  
  - Tiết kiệm điện năng  

---

### 1.3.5. Ứng dụng thực tế
- **CISC:**  
  - Ví dụ: Intel x86, AMD  
  - Dùng nhiều trong PC, laptop, server (cần khả năng tương thích và lệnh phức tạp)  

- **RISC:**  
  - Ví dụ: ARM, MIPS  
  - Dùng nhiều trong thiết bị di động, nhúng, IoT (cần hiệu suất cao, tiết kiệm năng lượng)  

---

## 1.4. Quan điểm cá nhân

Trong bối cảnh phát triển **hệ thống nhúng hiện nay**, **RISC (đặc biệt là ARM)** phù hợp hơn so với CISC bởi vì:  
- Tiêu thụ điện năng thấp  
- Hiệu suất/chi phí tốt  
- Hệ sinh thái phong phú  
- Khả năng mở rộng tốt  
