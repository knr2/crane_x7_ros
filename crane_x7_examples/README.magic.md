[English](README.en.md) | [日本語](README.md) | [マジック](README.magic.md)

# ロボットアームを使用した瞬間移動マジック

CRANE-X7で瞬間移動マジックをする際に使用するパッケージの説明です。

## システムの起動方法

CRANE-X7の制御信号ケーブルを制御用パソコンへ接続します。
Terminalを開き、`crane_x7_bringup`の`demo.launch`を起動します。
このlaunchファイルには次のオプションが用意されています。

- fake_execution (default: true)

実機を使用する/使用しない

### シミュレータを使う場合

実機無しで動作を確認する場合、
制御信号ケーブルを接続しない状態で次のコマンドを実行します。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=true
```

### 実機を使う場合

実機で動作を確認する場合、
制御信号ケーブルを接続した状態で次のコマンドを実行します。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```

ケーブルの接続ポート名はデフォルトで`/dev/ttyUSB0`です。
別のポート名(例: /dev/ttyUSB1)を使う場合は次のコマンドを実行します。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=false port:=/dev/ttyUSB1
```

### Gazeboを使う場合

次のコマンドで起動します。実機との接続やcrane_x7_bringupの実行は必要ありません。

```sh
roslaunch crane_x7_gazebo crane_x7_with_table.launch
```

## Run Examples

`demo.launch`を実行している状態で実行できます。

- [setup.py](#setup.py)
- [test1_pose_groupstate_example](#test1_pose_groupstate_example)

---

### setup

マジックの準備をするためのコードです。

表示される指示に従いマジックの準備をしてください。

次のコマンドを実行します。

```sh
rosrun crane_x7_examples setup.py
```

---

### pose_groupstate_example

マジックを行うためのコードです。

コード内で記述されている`home`に関する情報はSRDFファイル[crane_x7_moveit_config/config/crane_x7.srdf](../crane_x7_moveit_config/config/crane_x7.srdf)
に記載されています。

次のコマンドを実行します。

```sh
rosrun crane_x7_examples test1_pose_groupstate_example.py
```