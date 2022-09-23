# Mini_scope_base_on_Free_RTOS
此專案使用Stm32f446ret6開發板，及Nextion NX4832K035之人機介面進行開發。Nextion人機介面UI設置，可以透過Nextion官網所提供的編譯器進行設置。並藉由Free RTOS所提供的API，對SD卡儲存、人機介面操作、訊號產生器、FFT運算多個功能的多工整合及管理。

## Project Description
系統可以將ADC量測到之電壓波形、頻率、幅值等資訊，顯示在人機介面上，並且可以透過人機介面的操作暫停畫面、調整顯示波形(時間、大小、偏移)。顯示器另外一頁為額外功能，如FFT顯示、SD卡儲存，也是可以透過人機介面進行操作。

## Brief Architecture
![image](https://github.com/ZongWeiLin/Mini_scope_base_on_Free_RTOS/blob/main/flow_chart.png)
