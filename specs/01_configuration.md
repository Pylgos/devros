# 設定ファイル (Configuration)

devrosの設定は、プロジェクト共有用の `devros.toml` と、ローカル環境固有の `devros.local.toml` の2つのファイルで行います。

## 設定ファイルの優先順位
1.  `devros.local.toml`
    -   **役割**: 各開発者のローカル環境固有の設定（マシンのコア数に合わせた並列数、SSHの接続先など）。
    -   **Git管理**: `.gitignore` に追加し、コミットしないことを推奨。
    -   **動作**: `devros.toml` の設定を上書き（マージ）します。
2.  `devros.toml`
    -   **役割**: プロジェクト全体で共有する基本設定。
    -   **Git管理**: リポジトリにコミットして共有します。

## マージの仕様

`devros.local.toml` と `devros.toml` が両方存在する場合、以下のルールで設定がマージされます：

- **テーブル（セクション）**: 再帰的にマージされます
- **配列**: `devros.local.toml` の値が `devros.toml` の値を完全に上書きします（マージではなく置換）
- **プリミティブ値（文字列、数値、真偽値）**: `devros.local.toml` の値が優先されます

### マージ例

`devros.toml`:
```toml
[build]
jobs = 8
skip_packages = ["pkg_a", "pkg_b"]

[build.ament_cmake]
build_type = "Release"
cmake_args = ["-DFOO=1"]
```

`devros.local.toml`:
```toml
[build]
jobs = 16

[build.ament_cmake]
cmake_args = ["-DBAR=2"]
```

マージ結果:
```toml
[build]
jobs = 16                           # local.toml が優先
skip_packages = ["pkg_a", "pkg_b"]  # local.toml に定義がないため toml の値

[build.ament_cmake]
build_type = "Release"              # local.toml に定義がないため toml の値
cmake_args = ["-DBAR=2"]            # 配列は置換（マージではない）
```

## 設定ファイルの全体構成

以下は `devros.toml` の全体構成例です。各セクションの詳細は対応する仕様書を参照してください。

```toml
# ワークスペース設定
[workspace]
root = "."
build_dir = "build"
install_dir = "install"
state_dir = ".devros"

# ビルド設定（詳細: specs/04_build_system.md）
[build]
skip_packages = []
jobs = 16

[build.ament_cmake]
build_type = "RelWithDebInfo"
cmake_args = ["-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"]
export_compile_commands = true

# デプロイ設定（詳細: specs/06_deployment.md）
[deploy.local_dev]
type = "local"
target_dir = "install"

[deploy.robot_prod]
type = "remote"
method = "rsync"
target_dir = "/opt/robot_ws"
target = "admin@192.168.1.100"
post_deploy = "ssh {target} 'devros supervisor notify --packages {changed_packages}'"

# プロセス管理設定（詳細: specs/07_supervisor.md）
[supervisor.robot1.perception]
type = "launch"
package = "perception_bringup"
file = "perception.launch.py"
watch_packages = ["perception_nodes", "camera_driver"]

[supervisor.robot1.navigation]
type = "launch"
package = "nav2_bringup"
file = "navigation_launch.py"
watch_packages = ["nav2_nodes"]

# ファイル監視設定（詳細: specs/09_watch.md）
[watch]
watch_paths = ["src/"]
ignore_patterns = ["**/.git/**", "**/build/**", "**/.vscode/**"]
debounce_ms = 500
```

## 設定セクションの詳細

各設定セクションの詳細な仕様は、以下のドキュメントを参照してください：

- **`[workspace]`**: このファイル（基本設定のみ）
- **`[build]`**: `specs/04_build_system.md` - ビルドシステムの設定
- **`[deploy.<target>]`**: `specs/06_deployment.md` - デプロイメントの設定
- **`[supervisor.<group>.<process>]`**: `specs/07_supervisor.md` - プロセス管理の設定
- **`[watch]`**: `specs/09_watch.md` - ファイル監視の設定

## ワークスペース設定詳細

### `[workspace]`

```toml
[workspace]
root = "."
build_dir = "build"
install_dir = "install"
state_dir = ".devros"
```

#### `root`
- **型**: 文字列（パス）
- **必須**: いいえ（デフォルト: `"."`）
- **説明**: ワークスペースのルートディレクトリ。通常は `devros.toml` が配置されているディレクトリ（`"."`）を指定します。

#### `build_dir`
- **型**: 文字列（パス）
- **必須**: いいえ（デフォルト: `"build"`）
- **説明**: ビルドアーティファクトを保存するディレクトリ。
- **パス形式**: ワークスペースルートからの相対パス、または絶対パス。
- **用途**: パッケージごとのビルド中間ファイル（CMakeキャッシュ、オブジェクトファイルなど）
- **例**:
  - 相対パス: `"build"`, `".build"`
  - 絶対パス: `"/tmp/devros_build"`
  - 環境変数展開: `"$HOME/.cache/devros/build"` (実装で対応する場合)

#### `install_dir`
- **型**: 文字列（パス）
- **必須**: いいえ（デフォルト: `"install"`）
- **説明**: パッケージのインストール先ディレクトリ（開発環境用、シンボリックリンクを含む）。
- **パス形式**: ワークスペースルートからの相対パス、または絶対パス。
- **注意**: これは開発用のインストール先であり、デプロイ先（`[deploy.<target>].target_dir`）とは異なります。
- **例**:
  - 相対パス: `"install"`, `".install"`
  - 絶対パス: `"/opt/ros2_dev/install"`

#### `state_dir`
- **型**: 文字列（パス）
- **必須**: いいえ（デフォルト: `".devros"`）
- **説明**: devrosの内部状態ファイルを保存するディレクトリ。
- **パス形式**: ワークスペースルートからの相対パス、または絶対パス。
- **保存される情報**:
  - ビルドキャッシュ: `<state_dir>/cache/<package_name>/hash.json`
  - デプロイ履歴: `<state_dir>/deploy/<target_name>/last_deploy.json`
  - デプロイステージング: `<state_dir>/staging/<target_name>/`（実体化済みディレクトリ）
  - その他の内部状態
- **用途例**:
  - デフォルト（リポジトリ直下）: `".devros"`
  - ホームディレクトリ: `"$HOME/.cache/devros/my_project"`
  - プロジェクト固有の隠しディレクトリ: `".cache/devros"`
- **推奨**:
  - `build_dir` を一時ディレクトリ（`/tmp` など）に配置する場合でも、`state_dir` は永続化される場所に配置することを推奨します。
  - ステージングディレクトリを保持することで、次回のインクリメンタルデプロイが高速化されます。
  - `.gitignore` に追加して、バージョン管理から除外してください。

### ディレクトリ構造の例

デフォルト設定の場合、ワークスペースは以下のような構造になります：

```
workspace_root/
├── devros.toml
├── devros.local.toml (オプション)
├── src/                    # ソースコード
│   ├── package1/
│   └── package2/
├── build/                  # ビルドアーティファクト (build_dir)
│   ├── package1/
│   └── package2/
├── install/                # インストール先 (install_dir)
│   ├── package1/
│   └── package2/
└── .devros/                # devros内部状態 (state_dir)
    ├── cache/              # ビルドキャッシュ
    │   ├── package1/
    │   │   └── hash.json
    │   └── package2/
    │       └── hash.json
    ├── deploy/             # デプロイ履歴
    │   ├── local_dev/
    │   │   └── last_deploy.json
    │   └── robot_prod/
    │       └── last_deploy.json
    └── staging/            # デプロイステージング
        ├── local_dev/      # ローカルデプロイ用の実体化済みディレクトリ
        └── robot_prod/     # リモートデプロイ用の実体化済みディレクトリ
```
