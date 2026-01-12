# ビルドシステム (Build System)

## ビルドフロー
1.  **ワークスペーススキャン**: `src` 以下のパッケージを検出し、依存グラフを構築します。
2.  **トポロジカルソート**: 依存関係に基づき、正しいビルド順序を決定します。
3.  **キャッシュチェック**: 各パッケージについて、再ビルドが必要かどうかを判断します。
4.  **並列実行**: 独立したパッケージを並列にビルドします。`jobserver` を使用してシステム全体のリソース使用量を制御します。

## キャッシュ戦略
高速なインクリメンタルビルドを実現するため、Blake3ハッシュを使用した強力なキャッシュ機構を実装します。

### ハッシュ計算対象
以下の要素を組み合わせて、パッケージの一意なハッシュを計算します。
- **ソースコード**: パッケージディレクトリ内の全ファイル（`.gitignore` 等の除外ルールを適用）。
- **マニフェスト**: `package.xml` の内容。
- **ビルド設定**: `CMakeLists.txt` (ament_cmake) や `setup.py` (ament_python)。
- **依存関係**: 直接依存するパッケージのハッシュ値（推移的な変更を検知するため）。

### 判定ロジック
計算された現在のハッシュと、前回ビルド成功時のハッシュを比較します。一致する場合、そのパッケージのビルドはスキップされます。

### キャッシュの保存場所
- パッケージごとのハッシュ値は `<state_dir>/cache/<package_name>/hash.json` に保存されます。
- `<state_dir>` は `devros.toml` の `[workspace].state_dir` 設定値（デフォルト: `".devros"`）です。
- ファイル内容例:
  ```json
  {
    "hash": "blake3_hash_value",
    "timestamp": "2026-01-11T12:34:56Z",
    "status": "success"
  }
  ```

## Jobserver
Makeなどが採用しているJobserverプロトコルを利用し、サブプロセス（CMake/Make）を含めた全体の並列実行数を、ユーザーが指定した `jobs` リミット内に収めます。

## パッケージビルド詳細

### ament_cmake

CMakeを使用した標準的なROS 2パッケージのビルドプロセスです。

1.  **準備**
    *   **環境変数の設定**:
        *   **依存パッケージの読み込み**: 直接依存および推移的依存パッケージの環境フック（主に `.dsv` ファイル）を処理し、ビルド環境を構築します（詳細は `specs/05_environment_management.md` 参照）。これにより `AMENT_PREFIX_PATH`, `PYTHONPATH`, `LD_LIBRARY_PATH` 等が適切に設定されます。
        *   **ROS 2固有の調整**:
            *   `CMAKE_PREFIX_PATH` の同期: `AMENT_PREFIX_PATH` に含まれるパスを `CMAKE_PREFIX_PATH` にも追加（マージ）します。これは CMake の `find_package()` が `CMAKE_PREFIX_PATH` を参照するため必須です。

2.  **設定 (Configure)**
    CMakeを実行し、ビルド構成を生成します。
    ```bash
    cmake -B <build_dir> -S <source_dir> \
      -G "Unix Makefiles" \
      -DCMAKE_INSTALL_PREFIX=<install_dir> \
      -DAMENT_TEST_RESULTS_DIR=<build_dir>/test_results \
      -DBUILD_TESTING=ON \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo
    ```
    *   **シンボリックリンクモード (`--symlink-install`)**:
        `-DAMENT_CMAKE_SYMLINK_INSTALL=1` を追加します。これにより、CMakeのインストールターゲットがファイルをコピーする代わりにシンボリックリンクを作成するようになります。

3.  **ビルド (Build)**
    並列数 (`<jobs>`) を指定してコンパイルを実行します。
    ```bash
    cmake --build <build_dir> --parallel <jobs>
    ```

4.  **インストール (Install)**
    ビルド成果物をインストール先ディレクトリに配置します。
    ```bash
    cmake --install <build_dir>
    ```

5.  **後処理**
    *   **環境設定スクリプトの生成**:
        複雑なスクリプト生成を避け、ロジックを `devros` 本体に集約します。
        *   **`package.dsv`**: パッケージ固有の環境変数定義を記述したファイルを生成します。
        *   **`local_setup.sh`**: `devros` のサブコマンド（例: `devros env setup --package <name>`）を呼び出し、その出力を `eval` するだけのシンプルなスクリプトを生成します。
        *   **`setup.sh`**: ワークスペース全体を対象に `devros env setup` を呼び出すスクリプトを生成します。
        *   **前提**: この方式は実行環境にも `devros` がインストールされていることを前提とします。スタンドアロンなPythonスクリプト等によるセットアップ機能は、必要が生じた段階で検討・追加します。

### ament_python

`setup.py` を使用したPythonパッケージのビルドプロセスです。

1.  **準備**
    *   **インストール先のディレクトリ作成**:
        Pythonパッケージのインストール先となるライブラリパス（例: `<install_dir>/lib/python3.10/site-packages`）を事前に作成します。
    *   **インストール先のリダイレクト設定 (`sitecustomize.py`)**:
        `setup.py` はデフォルトでシステムのPythonパスにインストールしようとするため、これを回避する仕組みを構築します。
        ビルド用の一時ディレクトリ（`prefix_override`）に `sitecustomize.py` を生成し、このスクリプト内で `sys.prefix` と `sys.exec_prefix` を上書きして、インストール先ディレクトリ（`<install_dir>`）に向くように設定します。
    *   **シンボリックリンク用拡張コマンドの生成 (`--symlink-install` 時のみ)**:
        `data_files` をコピーではなくシンボリックリンクとしてインストールするための拡張コマンド `symlink_data` を定義する一時的なPythonパッケージを生成します。
        *   `distutils.command.install_data` を継承し、ファイルをコピーする代わりにシンボリックリンクを作成するようにオーバーライドしたクラスを作成します。
        *   このクラスを `distutils.commands` エントリーポイントに登録するメタデータを生成します。
    *   **環境変数の設定 (`PYTHONPATH`)**:
        *   **依存パッケージの読み込み**: `.dsv` ファイル等を処理し、依存関係のパス（`PYTHONPATH` 等）を環境変数に反映させます。
        *   **`PYTHONPATH` の調整**:
            `setup.py` 実行用に `PYTHONPATH` を以下の優先順位で再構成します。
            1.  `sitecustomize.py` が配置されたディレクトリ。
            2.  シンボリックリンク用拡張コマンドが生成されたディレクトリ（必要な場合）。
            3.  インストール先のライブラリパス（自分自身の依存解決用）。
            4.  依存パッケージ読み込みで設定された（既存の） `PYTHONPATH`。

2.  **実行**
    *   **標準モード**:
        ```bash
        python3 setup.py egg_info --egg-base <build_dir>
        python3 setup.py build --build-base <build_dir>/build \
          install --prefix <install_dir> \
                  --record <build_dir>/install.log \
                  --single-version-externally-managed
        ```
    *   **シンボリックリンクモード (`--symlink-install`)**:
        *   ソースコード、`package.xml`、`setup.py` 等をビルドディレクトリへシンボリックリンクします。
        *   `develop` コマンドと、生成した拡張コマンド `symlink_data` を実行します：
            ```bash
            python3 setup.py develop --prefix <install_dir> \
              --build-directory <build_dir>/build \
              --editable --no-deps \
              symlink_data
            ```

3.  **後処理**
    *   **メタデータの配置**:
        *   `package.xml` がインストール先に存在しない場合、コピーまたはリンクを作成します。
        *   Amentリソースインデックスへの登録（`share/ament_index/resource_index/packages/<name>` マーカーファイルの作成）を行います。
    *   **環境設定スクリプトの生成**:
        `ament_cmake` と同様に、`package.dsv` を生成し、`devros` を利用した `local_setup.sh` を配置します。
        *   生成する `package.dsv` には、`AMENT_PREFIX_PATH` の設定に加え、Pythonパッケージ特有の `PYTHONPATH` へのパス追加（`lib/pythonX.Y/site-packages`）を含めます。

## ビルド設定 (devros.toml)

ビルドシステムの動作は、`devros.toml` の `[build]` セクションで制御します。

```toml
[build]
# ビルド対象から常に除外するパッケージ名のリスト
skip_packages = ["broken_package", "experimental_pkg"]

# 並列ビルドのジョブ数
# デフォルトはマシンの論理コア数
jobs = 16

# ament_cmake パッケージ固有の設定
[build.ament_cmake]
# CMAKE_BUILD_TYPE (例: "Debug", "Release", "RelWithDebInfo")
build_type = "RelWithDebInfo"

# 全パッケージのビルド時に共通して渡されるCMake引数
cmake_args = ["-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"]

# compile_commands.json を出力するかどうか (LSP等で使用)
export_compile_commands = true
```

### 設定項目詳細

#### `[build]`

##### `skip_packages`
- **型**: 文字列配列（パッケージ名）
- **必須**: いいえ（デフォルト: `[]`）
- **説明**: ビルド対象から常に除外するパッケージ名のリスト。依存関係の解決からも除外されます。
- **用途**:
  - ビルドが壊れているパッケージ
  - 実験的なパッケージ
  - 特定の環境でのみ必要なパッケージ

##### `jobs`
- **型**: 整数
- **必須**: いいえ（デフォルト: システムの論理コア数）
- **説明**: 並列ビルドのジョブ数。Jobserverを通じてサブプロセス（CMake/Make）を含めた全体の並列度を制御します。
- **推奨**: `devros.local.toml` でマシンごとに設定することを推奨します。

#### `[build.ament_cmake]`

ament_cmakeパッケージのビルドに関する設定です。

##### `build_type`
- **型**: 文字列
- **必須**: いいえ（デフォルト: `"RelWithDebInfo"`）
- **説明**: CMakeのビルドタイプ（`CMAKE_BUILD_TYPE`）。
- **有効な値**:
  - `"Debug"`: デバッグ情報あり、最適化なし
  - `"Release"`: 最適化あり、デバッグ情報なし
  - `"RelWithDebInfo"`: 最適化あり、デバッグ情報あり（推奨）
  - `"MinSizeRel"`: サイズ最適化

##### `cmake_args`
- **型**: 文字列配列
- **必須**: いいえ（デフォルト: `[]`）
- **説明**: 全ament_cmakeパッケージのビルド時に共通して渡されるCMake引数。
- **例**:
  - `"-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"`: compile_commands.jsonの生成
  - `"-DBUILD_TESTING=OFF"`: テストビルドの無効化
  - `"-DCMAKE_CXX_FLAGS=-Wall"`: コンパイラフラグの追加

##### `export_compile_commands`
- **型**: 真偽値
- **必須**: いいえ（デフォルト: `false`）
- **説明**: `compile_commands.json` を出力するかどうか。
- **動作**: `true` の場合、各パッケージの `compile_commands.json` をワークスペースルートにマージして統合ファイルを生成します。LSP（Language Server Protocol）サーバーがこのファイルを参照することで、IDE上でのコード補完や定義ジャンプが可能になります。
