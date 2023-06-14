# 概要

Kohakuのgazebo用の設定および確認のためのlaunchファイルが格納されているROSパッケージです。

# パッケージの構成

- launch launchファイルが格納されたフォルダ
- config gazebo用のコントローラ設定が格納されたフォルダ
- world  シミュレーションで読み込むworldファイルが格納されたフォルダ

# Launch Files

## kohaku_controller.launch

gazebo用のコントローラを起動するためのlaunchファイル

## kohaku_model4_gazebo.launch

empty.world上にkohakuを配置して起動するためのlaunchファイル

# モデルの確認方法

以下のコマンドで確認できる。

``` bash
$ roslaunch kohaku_gazebo kohaku_model4_gazebo.launch
```
