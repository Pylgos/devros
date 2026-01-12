# 環境変数管理 (Environment Management)

[reference_colcon_environment_management.md](./reference_colcon_environment_management.md)も参考にしてください。

## Devros における実装方針

devrosは、`colcon` の低速なPythonベースのセットアップスクリプト (`_local_setup_util_sh.py`) を排除し、Rustによる高速かつ安全な環境変数管理を提供します。

### 1. コア戦略

#### インメモリ計算
ビルド時やコマンド実行時に、Rust内部で環境変数の状態を計算します。
- **発見**: `share/colcon-core/packages/` のマーカーファイルを利用して、ワークスペース内のパッケージを網羅します。
- **ソート**: 依存グラフのトポロジカルソート順を計算します。
- **適用**: 各パッケージの `.dsv` (Dynamic Source Value) ファイル定義をパースし、環境変数への操作 (Set, Append, Prepend) をメモリ上でシミュレーションします。

#### シェルスクリプト生成 (`devros env`)
環境変数を適用するためのシェルコマンドを生成するCLIコマンドを提供します。

```bash
devros env shell --shell bash
```

このコマンドは、計算された環境変数を設定するための一連の `export` コマンドなどを標準出力に出力します。これにより、複雑なロジックをRust側に隠蔽し、シェルスクリプトを単純化します。

#### 軽量なセットアップスクリプト
`install/setup.bash` は、`devros env` コマンドを呼び出すだけの軽量なラッパーとして生成されます。

```bash
# install/setup.bash の例
eval "$(devros env shell --shell bash)"
```

これにより、`source install/setup.bash` の実行時間が大幅に短縮されます。

## .dsv (Dynamic Source Value) ファイル
環境変数の変更操作を定義するファイル形式です。各パッケージのビルド成果物として生成されるか、またはdevros内部で生成されます。

### ファイル形式
- **デリミタ**: セミコロン (`;`) 区切り
- **コメント**: `#` で始まる行は無視される
- **空行**: 無視される
- **構造**: `type;arg1;arg2;...`

### サポートされる操作 (Operations)

#### 1. `prepend-non-duplicate`
環境変数の先頭に値を追加します（重複排除あり）。
- **形式**: `prepend-non-duplicate;VARIABLE_NAME;value1;value2;...`
- **挙動**:
    - `value` が空の場合、パッケージのインストールプレフィックスパスを使用します。
    - `value` が相対パスの場合、プレフィックスパスからの相対パスとして解決されます。
    - 既に環境変数に値が含まれている場合は追加しません。

#### 2. `prepend-non-duplicate-if-exists`
`prepend-non-duplicate` と同様ですが、指定されたパスが存在する場合のみ追加します。
- **形式**: `prepend-non-duplicate-if-exists;VARIABLE_NAME;value`

#### 3. `append-non-duplicate`
環境変数の末尾に値を追加します（重複排除あり）。
- **形式**: `append-non-duplicate;VARIABLE_NAME;value`

#### 4. `set`
環境変数を特定の値に設定します。
- **形式**: `set;VARIABLE_NAME;value`
- **挙動**:
    - `value` が空の場合、プレフィックスパスを使用します。
    - `value` が相対パスの場合、`prefix/value` が存在すればその絶対パスを使用します。存在しなければ元の値をそのまま使用します。

#### 5. `set-if-unset`
環境変数が未設定の場合のみ値を設定します。
- **形式**: `set-if-unset;VARIABLE_NAME;value`

#### 6. `source`
他のスクリプトや `.dsv` ファイルを読み込みます。
- **形式**: `source;path/to/script`
- **挙動**:
    - 指定されたパス（拡張子なし、または `.dsv` 等）に基づいて、再帰的に処理を行います。
    - `.dsv` ファイルが見つかった場合、それをパースして処理します。
    - シェルスクリプトが見つかった場合（例: `.bash`, `.sh`）、それを `source` するコマンドを生成します。

### 実装上の注意
- Rust側で `.dsv` をパースする際は、`colcon-core` の `prefix_util.py.em` (`process_dsv_file` 関数など) と同等のロジックを実装する必要があります。特に、相対パスの解決ロジックと重複排除ロジックに注意してください。
- `ament_cmake` パッケージ以外の互換性（例: catkinパッケージ）も考慮し、`catkin_env_hook` などの仕組みも将来的にはサポートが必要になる可能性がありますが、当面は ROS 2 (`ament`) 標準に準拠します。
