# NX2 Proコントローラーエミュレーション (Raspberry Pi Pico + TinyUSB)

Raspberry Pi Pico 上で Nintendo Switch2 (NX2) の Pro コントローラーをエミュレートする最小構成のサンプルです。`pico-sdk` と TinyUSB を利用し、USB HID デバイスとして Switch2 向けのレポートを送信します。

## 仕組みの概要
- `src/usb_descriptors.c` に NX2 Pro コントローラーを模した HID レポートディスクリプタとデバイスディスクリプタを定義しています (VID 0x057E / PID 0x2069 / BCD 0x0300)。
- HID レポートディスクリプタ上は入力 0x30 / 出力 0x21 / Feature 0x80 の 64B レポート構成ですが、**実際のコマンドは `commands.md` に従うベンダー独自形式**です。デスクリプタはあくまで列挙用と割り切っています。
- `src/main.c` では TinyUSB を初期化し、Switch2 からの出力レポート 0x21 を `commands.md` のコマンドとして解釈します。プロトコルバージョン(0x01)/デバイス情報(0x02)/ユニーク ID(0x03)/SHIP モード(0x04)/ポーリング開始(0x10)/停止(0x11)/LED(0x12/0x13)/ランブル(0x14)/IMU 有効化(0x15)/キャリブレーション(0x20) を解析し、コマンド ID とステータスを先頭に据えた 64 バイトの応答を返します。ポーリング開始後は 200Hz 程度でニュートラル入力を送り続けます。

## ビルド手順
1. `PICO_SDK_PATH` を有効な `pico-sdk` ディレクトリに設定します。
2. ビルドディレクトリを作成し、CMake で生成します。

```bash
mkdir -p build
cd build
cmake ..
cmake --build .
```

生成される `nx2_pro_controller.uf2` を Pico にドラッグ&ドロップすると、USB で Pro コントローラーとして列挙されます。

## 留意点
- レポート送信はニュートラル状態のみです。ボタンやスティックを操作したい場合は `idle_payload` を更新してください。
- ランブル・LED・IMU のコマンドは入力レポートで即座に ACK を返し、Feature レポート 0x80 にも同じ ACK を保持します。実際の挙動を変えたい場合は `respond_led_or_rumble` などのハンドラを拡張してください。
- 実機での認証仕様変更により VID/PID やディスクリプタを調整する必要が生じる可能性があります。
