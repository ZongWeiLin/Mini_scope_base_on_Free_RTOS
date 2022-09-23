# Mini_scope_base_on_Free_RTOS
此專案使用Stm32f446ret6開發板，及Nextion NX4832K035之人機介面進行開發。Nextion人機介面UI設置，可以透過Nextion官網所提供的編譯器進行設置。並藉由Free RTOS所提供的API，對SD卡儲存、人機介面操作、訊號產生器、FFT運算多個功能的多工整合及管理。

## Project Description
系統可以將ADC量測到之電壓波形、頻率、幅值等資訊，顯示在人機介面上，並且可以透過人機介面的操作暫停畫面、調整顯示波形(時間、大小、偏移)。顯示器另外一頁為額外功能，如FFT顯示、SD卡儲存，也是可以透過人機介面進行操作。

## Brief Architecture
![image](https://github.com/ZongWeiLin/Mini_scope_base_on_Free_RTOS/blob/main/flow_chart.png)
* OS callback
   * 用於進行電壓(ADC)採樣。
   * 傳輸採樣電壓資訊。
   * 在FFT運算需要的資料夠時，釋放Semaphore與FFT運算的任務進行同步。
* Scope Task
  * 確認停止鈕以及FFT顯示紐的狀態，決定是否要傳輸資料至顯示屏。
  * 傳輸量測到的資料至顯示屏。
    *  Note : 在資料傳輸時，會透過Mutex對UART進行管理，以免當前任務傳輸的資料被其他任務給蓋掉。
* Data Task
  * 確認停止鈕狀態，決定是否要傳輸資料至顯示屏。
  * 傳輸量測到的資料至顯示屏。
    *  Note : 在資料傳輸時，會透過Mutex對UART進行管理，以免當前任務傳輸的資料被其他任務給蓋掉。
* FFT Task
  * 透過是否有Semaphore確認資料是否準備好了。
  * 獲取Semaphore後，由Queue接收要FFT運算所需的資料，之後執行FFT運算。
* SD Task
  * 確認是否有無要儲存資料。
  * 透過Queue接收要儲存進SD卡的資料。
## Files
* Core/Src
  * main.c:主程式，RTOS的所有任務皆寫在此。
  * stm32f4xx_it.c:設定Interrupt Handler
* Driver:ST官方提供的Hal函式庫
* FATFS/Target
  * user_diskio.c:需將SD_SPI_FATFS.c完成的檔案操作底層IO API移植進此檔案
* Middlewares/Third_Party
  * FATFS:File Allocation Table File System 的source code
  * FreeRTOS : FreeRTOS 的source code
* Mini_scope_base_on_Free_RTOS.ioc:開發板的腳位、時脈設定
