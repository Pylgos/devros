# リファレンス: ColconとAmentの環境変数管理

本書は、`colcon`、`ament_cmake`、`ament_python`が環境変数とシェルセットアップスクリプトをどのように管理しているかを詳細に記述します。これは、リファレンス実装（`colcon-core`、`colcon-ros`、`ament_cmake`など）の詳細な分析に基づいています。

## 1. 概要

環境管理システムは、ワークスペースの連鎖（オーバーレイ）を可能にし、パッケージをインストールせずに（一部の場合）、あるいはインストール後に使用できるように設計されています。

主要なコンポーネントは以下の通りです：
1.  **`setup.sh`**: ワークスペースのエントリーポイント。親ワークスペースの連鎖（chaining）を処理します。
2.  **`local_setup.sh`**: *現在の*ワークスペース専用のセットアップスクリプト。パッケージ固有のスクリプトをsourceします。
3.  **`_local_setup_util_sh.py`**: パッケージをsourceするトポロジカル順序（依存関係順）を決定するヘルパースクリプト。
4.  **`package.dsv` / `package.sh`**: パッケージ固有の環境記述/スクリプト。
5.  **Environment Hooks (環境フック)**: パッケージや拡張機能によって登録される特定の変更（例：`PATH`や`CMAKE_PREFIX_PATH`の変更）。

## 2. シェルスクリプト生成 (Colcon側)

`colcon`は、インストールディレクトリにトップレベルのシェルスクリプト（`setup.sh`、`local_setup.sh`）とパッケージ固有のスクリプト（`package.sh`、`package.dsv`）を生成します。

### 2.1. ワークスペースのセットアップ (`setup.sh`)
- **ソース**: `colcon_core/shell/template/prefix_chain.sh.em`
- **機能**:
    1.  親ワークスペースの`setup.sh`があれば、それをsourceします。
    2.  現在のワークスペースの`local_setup.sh`をsourceします。

### 2.2. ローカルワークスペースのセットアップ (`local_setup.sh`)
- **ソース**: `colcon_core/shell/template/prefix.sh.em`
- **機能**:
    1.  `COLCON_CURRENT_PREFIX`をインストールパスに設定します。
    2.  現在のプレフィックスを`COLCON_PREFIX_PATH`に追加します。
    3.  `_local_setup_util_sh.py`を実行し、トポロジカル順にsourceすべきパッケージスクリプトのリストを取得します。
    4.  返されたスクリプトをsourceします。

### 2.3. トポロジカル順序付け (`_local_setup_util_sh.py`)
- **ソース**: `colcon_core/shell/template/prefix_util.py.em`
- **機能**:
    1.  `share/colcon-core/packages`（マージインストール時）またはサブディレクトリ（分離インストール時）をスキャンしてパッケージを探します。
    2.  これらのマーカーファイルから実行時の依存関係を読み取ります。
    3.  トポロジカルソートを実行します。
    4.  各パッケージについて、`share/<pkg>/package.dsv`を確認します。
    5.  `package.dsv`を解析し、シェルコマンド（ファイルのsource、変数の設定/追加）を生成します。

### 2.4. パッケージスクリプト (`package.dsv` / `package.sh`)
- **ソース**: `colcon_core/shell/template/package.dsv.em` / `package.sh.em`
- **機能**:
    - `package.dsv`は、`_local_setup_util_sh.py`が使用する主要なインターフェースです。
    - 操作のリスト（`source`, `set`, `set-if-unset`, `prepend-non-duplicate`, `append-non-duplicate`）が含まれます。
    - `colcon`は"Environment Hooks"を集約してこのファイルを生成します。

## 3. Environment Hooks (環境フック)

フックは環境を変更するためのメカニズムです。以下によって登録されます：
1.  **Colcon Extensions**: ビルド/インストールフェーズで実行される`colcon`内のプラグイン。
2.  **Build System**: 独自のフックを生成するビルドシステム（例：`ament_cmake`）。

### 3.1. 標準Colconフック
`colcon-core`とプラグインは標準的なフックを定義しています：
- **`CMAKE_PREFIX_PATH`**: `CmakePrefixPathEnvironment` (`colcon-cmake`内)によって処理されます。`*Config.cmake`をスキャンします。
- **`PYTHONPATH`**: `PythonPathEnvironment` (`colcon-core`内)によって処理されます。`site-packages` / `dist-packages`をスキャンします。
- **`PATH`**: `PathEnvironment` (`colcon-core`内)によって処理されます。`bin`ディレクトリを確認します。

### 3.2. Ament CMakeの統合
`ament_cmake`パッケージは環境管理をCMakeロジックに委譲し、CMakeロジックが*パッケージのshareディレクトリ内に*`local_setup.sh`を生成します。

- **メカニズム**:
    1.  `ament_cmake`は`share/<pkg>/local_setup.sh`（および`.bash`、`.zsh`など）と`share/<pkg>/package.dsv`を生成します。
    2.  この`local_setup.sh`は、CMakeマクロ`ament_environment_hooks()`を通じて登録されたフックをsourceします。
    3.  **重要な点**: `colcon-ros`の`AmentCmakeBuildTask`はこれを検出し、`colcon`の`package.dsv`に特別なフックを登録します：
        ```text
        source;share/<pkg_name>/local_setup.sh
        ```
    - つまり、`colcon`の`package.dsv`は`ament_cmake`の`local_setup.sh`にジャンプします。

- **標準Amentフック** (`ament_cmake_core`によって登録):
    - **`AMENT_PREFIX_PATH`**: パッケージプレフィックスを先頭に追加します。
    - **`LD_LIBRARY_PATH`**: (`ament_cmake_environment_hooks_package_hook`経由) `lib`を先頭に追加します。

### 3.3. Ament Pythonの統合
`ament_python`パッケージは`setup.py`経由でビルドされますが、`colcon`によって調整されます。

- **メカニズム**:
    - `colcon-ros`の`AmentPythonBuildTask`（`PythonBuildTask`を拡張）が手動でフックを登録します。
    - **`AMENT_PREFIX_PATH`**: `AmentPythonBuildTask`によって明示的に追加されます。
    - **`PYTHONPATH`**, **`PATH`**: 標準的な`colcon`拡張（`PythonPathEnvironment`など）によって処理されます。
    - `ament_python`パッケージは`share/<pkg>`内に`local_setup.sh`を**生成しません**。すべては`colcon`の`package.dsv`内にあります。

## 4. 主要な環境変数

| 変数 | 管理者 | 目的 |
| :--- | :--- | :--- |
| `COLCON_PREFIX_PATH` | `colcon` (`setup.sh`) | ワークスペースプレフィックスのリスト。 |
| `COLCON_CURRENT_PREFIX` | `colcon` (`local_setup.sh`) | 現在のワークスペースプレフィックス。 |
| `AMENT_PREFIX_PATH` | `ament_cmake` (hook) / `AmentPythonBuildTask` | Amentパッケージプレフィックスのリスト。ROS 2ツールがパッケージ/リソースを見つけるために使用します。 |
| `CMAKE_PREFIX_PATH` | `colcon-cmake` (extension) | CMakeの`find_package`で使用されます。 |
| `PYTHONPATH` | `colcon-core` (extension) | Pythonモジュールの検索パス。 |
| `LD_LIBRARY_PATH` | `ament_cmake` (hook) | 共有ライブラリの検索パス。 |
| `PATH` | `colcon-core` (extension) | 実行ファイルの検索パス。 |

## 5. ファイル形式

### 5.1. `.dsv` (Delimiter Separated Values)
シェルに依存しない方法で環境操作を記述するために使用されます。
形式: `type;argument[;argument...]`

タイプ:
- `source;path/to/script` (パスは絶対パスでなければインストールプレフィックスからの相対パス)
- `set;VAR;VALUE`
- `set-if-unset;VAR;VALUE`
- `prepend-non-duplicate;VAR;VALUE`
- `append-non-duplicate;VAR;VALUE`

例 `package.dsv` (ament_cmake):
```text
source;share/my_pkg/local_setup.sh
```

例 `local_setup.dsv` (ament_cmake 内部):
```text
prepend-non-duplicate;AMENT_PREFIX_PATH;
prepend-non-duplicate;LD_LIBRARY_PATH;lib
```
