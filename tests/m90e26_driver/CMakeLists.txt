# 指定 CMake 最低版本要求
cmake_minimum_required(VERSION 3.20.0)

# 尋找 Zephyr 的建置系統，這是所有 Zephyr 應用的標準起始點
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# 為這個測試應用程式設定一個專案名稱
project(m90e26_driver_test)

# 將測試的原始碼檔案 (src/main.c) 加入到名為 "app" 的目標中
target_sources(app PRIVATE src/main.c)
