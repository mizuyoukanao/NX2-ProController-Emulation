# NX2 Proコントローラーエミュレーション (Raspberry Pi Pico + TinyUSB)
Raspberry Pi Pico 上で Nintendo Switch2 (NX2) の Pro コントローラー/JoyCon2 (R)をエミュレートする最小構成のサンプルです。`pico-sdk` と TinyUSB を利用し、USB HID デバイスとして Switch2 向けのレポートを送信します。

## 仕組みの概要
- `src/usb_descriptors.c` に NX2 Pro コントローラー/JoyCon2 (R)を模した HID レポートディスクリプタとデバイスディスクリプタを定義しています (VID 0x057E / PID 0x2069,0x2066,0x2067 / BCD 0x0200)。
- `src/main.c` では TinyUSB を初期化し、Switch2 からの出力レポート を [この資料](https://github.com/ndeadly/switch2_controller_research/blob/master/commands.md) のコマンドとして解釈します。

## ビルド手順
1. `PICO_SDK_PATH` を有効な `pico-sdk` ディレクトリに設定します。
2. ビルドディレクトリを作成し、CMake で生成します。

```bash
mkdir -p build
cd build
cmake ..
cmake --build .
```

生成される `nx2_pro_controller.uf2` を Pico にドラッグ&ドロップすると、USB で NX2 Pro コントローラー/JoyCon2 (R)として列挙されます。

## 留意点
- レポート送信はBOOTSELボタンによるAボタンのON/OFFのみです。他のボタンやスティックを操作したい場合は `idle_payload` を更新してください。

## 使用ライブラリ
https://github.com/halloweeks/AES-128-ECB
