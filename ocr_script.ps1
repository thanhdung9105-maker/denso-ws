[Windows.Globalization.Language, Windows.Foundation, ContentType = WindowsRuntime] | Out-Null
[Windows.Graphics.Imaging.BitmapDecoder, Windows.Foundation, ContentType = WindowsRuntime] | Out-Null
[Windows.Media.Ocr.OcrEngine, Windows.Foundation, ContentType = WindowsRuntime] | Out-Null

function Run-Ocr($imagePath) {
    $absPath = (Resolve-Path $imagePath).Path
    $fileTask = [Windows.Storage.StorageFile]::GetFileFromPathAsync($absPath)
    $file = $fileTask.GetAwaiter().GetResult()
    $streamTask = $file.OpenAsync([Windows.Storage.FileAccessMode]::Read)
    $stream = $streamTask.GetAwaiter().GetResult()
    $decoderTask = [Windows.Graphics.Imaging.BitmapDecoder]::CreateAsync($stream)
    $decoder = $decoderTask.GetAwaiter().GetResult()
    $bmpTask = $decoder.GetSoftwareBitmapAsync()
    $softwareBitmap = $bmpTask.GetAwaiter().GetResult()
    $ocrEngine = [Windows.Media.Ocr.OcrEngine]::TryCreateFromUserProfileLanguages()
    if ($null -eq $ocrEngine) {
        $ocrEngine = [Windows.Media.Ocr.OcrEngine]::TryCreateFromLanguage([Windows.Globalization.Language]::new('en-US'))
    }
    $ocrTask = $ocrEngine.RecognizeAsync($softwareBitmap)
    $ocrResult = $ocrTask.GetAwaiter().GetResult()
    return $ocrResult.Text
}

for ($i = 1; $i -le 4; $i++) {
    Write-Host "================ PAGE $i ================"
    $text = Run-Ocr "C:\Users\thanh\denso_ws\gwo_page_$i.png"
    Write-Host $text
}

