# デプロイメント (Deployment)

devrosは、開発環境での利便性と、本番環境へのポータビリティを両立させるデプロイ機能を提供します。

## Materialization (実体化)

`devros deploy` を実行すると、開発用のシンボリックリンクを含んだ `install` ディレクトリから、ポータブルな（再配置可能な）配布用ディレクトリを作成します。

このプロセスにより、ソースコードディレクトリへの依存を排除し、ビルド環境とは異なるパス構成の環境（本番機など）でも動作するアーティファクトを生成します。

### ステージングディレクトリ

Materialization処理は、まずステージングディレクトリで実体化を行います。

- **配置場所**: `<state_dir>/staging/<target_name>/`
  - `<state_dir>` は `devros.toml` の `[workspace].state_dir` 設定値（デフォルト: `".devros"`）
- **用途**:
  - ローカルデプロイ: ステージング完了後、`target_dir` にコピーまたは移動
  - リモートデプロイ: ステージング完了後、rsyncでリモートへ転送
- **永続性**: デプロイ成功後も保持され、次回のインクリメンタルデプロイで再利用可能（rsyncの差分転送が効率化）
- **クリーンアップ**: 明示的に削除する場合は `devros clean --deploy` または手動削除

### 1. ディレクトリ構造の統合 (Merge)
- すべての対象パッケージのインストール成果物を、単一のターゲットディレクトリにマージします。これにより、`Isolated` なインストール環境から、`Merged` な配布環境（`/usr` や `/opt/ros/xxx` に似た構成）へ変換します。
- **衝突解決**: 同一パスにファイルが存在する場合、依存関係グラフの下流（より上位のパッケージ）が上書きします。衝突時は警告ログを出力します。

### 2. ファイルのコピーとシンボリックリンクの解決
- **通常ファイル**: そのままコピーします。
- **シンボリックリンク**:
    - **内部リンク**: リンク先がターゲットディレクトリ内部（マージ後）を指す場合、相対パスのシンボリックリンクとして保持します（絶対パスの場合は相対パスに変換します）。
    - **外部リンク**: リンク先がターゲットディレクトリ外部（ソースコード領域など）を指す場合、リンクの実体（ターゲットファイル/ディレクトリ）をコピーします（Dereference）。これにより、ソースコードへの依存を断ち切ります。

### 3. Pythonパッケージのフリーズ (Freezing)
`ament_python --symlink-install` によって生成された `.egg-link` ベースの環境を、通常のインストール構成（`site-packages` への配置）に変換します。

1.  **`.egg-link` の検出**: `lib/pythonX.Y/site-packages/` 内の `.egg-link` ファイルを探索します。
2.  **ソースのコピー**: `.egg-link` が指すソースディレクトリの内容を、ターゲットの `site-packages/<package_name>` にコピーします。
3.  **メタデータのコピー**: 対応する `*.egg-info` ディレクトリをソース側から探し、ターゲットへコピーします。
4.  **クリーンアップ**:
    - ターゲット内の `.egg-link` ファイルを削除します。
    - `easy-install.pth` から、該当するソースディレクトリへのパス行を削除します。
5.  **バイトコードコンパイル**: `.py` ファイルをコンパイルし、`.pyc` を生成します。これにより、ターゲット環境での初回起動時のコンパイル処理（および書き込み権限エラー）を回避します。
6.  **Shebangの書き換え**: `bin/` 以下のPythonスクリプトの Shebang (`#!...`) がビルド環境のPython絶対パスになっている場合、`/usr/bin/env python3` 等の汎用的なものに書き換えます。

### 4. 環境設定スクリプトの生成
デプロイ先環境にも `devros` がインストールされていることを前提とします。

- **生成ファイル**: `setup.bash`, `setup.sh` (POSIX)
- **内容**:
    - `devros` コマンドを呼び出し、環境変数設定スクリプトを生成・評価（`eval`）するラッパーです。
    - 例: `eval "$(devros env shell --prefix "$(dirname "$0")")"`
    - これにより、デプロイ先の `devros` のロジックに従って、`AMENT_PREFIX_PATH` や `LD_LIBRARY_PATH` が動的に設定されます。パッケージごとの `.dsv` ファイルなども適切に処理されます。

## 変更パッケージの検出

デプロイ時には、前回のデプロイから変更されたパッケージを検出し、`{changed_packages}` プレースホルダに渡します。

### 検出方法

デプロイシステムは、ビルドシステムのキャッシュ機構（Blake3ハッシュベース）を活用して変更を検出します。

1. **デプロイ履歴の記録**
   - デプロイ実行時に、各ターゲットごとの状態ファイルを保存します。
   - 保存場所: `<state_dir>/deploy/<target_name>/last_deploy.json`
     - `<state_dir>` は `devros.toml` の `[workspace].state_dir` 設定値（デフォルト: `".devros"`）
   - 保存内容:
     ```json
     {
       "timestamp": "2026-01-11T12:34:56Z",
       "packages": {
         "perception_nodes": "blake3_hash_value_1",
         "camera_driver": "blake3_hash_value_2",
         "nav2_nodes": "blake3_hash_value_3"
       }
     }
     ```

2. **変更の判定**
   - 現在のビルド状態（各パッケージのBlake3ハッシュ）と、前回デプロイ時のハッシュを比較します。
   - ハッシュが異なる、または新規パッケージの場合、「変更あり」と判定します。
   - 判定に使用するハッシュは、ビルドシステムのキャッシュ計算と同一のもの（ソースコード、マニフェスト、ビルド設定、依存関係を含む）です。

3. **変更パッケージリストの生成**
   - 変更があったパッケージ名をリストアップします。
   - `{changed_packages}` プレースホルダには、スペース区切りの文字列として渡されます。
   - 例: `"perception_nodes camera_driver"`

### 初回デプロイ

前回のデプロイ履歴が存在しない場合（初回デプロイ）：
- すべてのパッケージが「変更あり」として扱われます。
- `{changed_packages}` には全パッケージ名が含まれます。

### 部分デプロイ

`devros deploy <target> --packages pkg1 pkg2` のように特定のパッケージのみをデプロイする場合：
- 指定されたパッケージのみが変更判定の対象となります。
- `{changed_packages}` には、指定されたパッケージのうち実際に変更があったもののみが含まれます。

## Remote Deployment
`devros deploy <remote_target>` を実行すると、SSH/rsync を使用して上記で Materialize されたディレクトリをリモートマシンへ転送します。

- **転送**: `rsync` を使用し、チェックサム比較等を用いて差分のみを効率的に転送します。
- **変更検出**: 前回のデプロイ状態と比較して、変更されたパッケージを特定します（上記「変更パッケージの検出」参照）。
- **SSHフック**: `devros.toml` の `post_deploy` 設定に基づき、転送完了後にリモート側でコマンドを実行します。
    - 例: `systemctl --user restart my_robot_service`
    - `{changed_packages}` プレースホルダには、変更されたパッケージ名がスペース区切りで渡されます。

## デプロイ設定 (devros.toml)

デプロイ先は `devros.toml` の `[deploy.<target_name>]` セクションで定義します。

### ローカルデプロイの例

```toml
[deploy.local_dev]
type = "local"
target_dir = "install"
```

### リモートデプロイの例

```toml
[deploy.robot_prod]
type = "remote"
method = "rsync"
target_dir = "/opt/robot_ws"
target = "admin@192.168.1.100"
post_deploy = "ssh {target} 'devros supervisor notify --packages {changed_packages}'"
```

### 設定項目詳細

#### 共通設定

##### `type`
- **型**: 文字列
- **必須**: はい
- **説明**: デプロイの種類。
- **有効な値**:
  - `"local"`: ローカルマシン上のディレクトリへデプロイ
  - `"remote"`: リモートマシンへSSH/rsyncでデプロイ

#### ローカルデプロイ (`type = "local"`)

##### `target_dir`
- **型**: 文字列（パス）
- **必須**: はい
- **説明**: デプロイ先のディレクトリパス。
- **パス形式**:
  - ワークスペースルートからの相対パス（例: `"install"`, `"deploy/local"`）
  - 絶対パス（例: `"/opt/ros2_ws"`）

#### リモートデプロイ (`type = "remote"`)

##### `method`
- **型**: 文字列
- **必須**: はい
- **説明**: 転送方法。
- **有効な値**:
  - `"rsync"`: rsyncによる差分転送（現在のところこれのみサポート）

##### `target_dir`
- **型**: 文字列（パス）
- **必須**: はい
- **説明**: リモートマシン上のデプロイ先ディレクトリ。
- **パス形式**: 絶対パス（例: `"/opt/robot_ws"`, `"/home/robot/workspace"`）

##### `target`
- **型**: 文字列
- **必須**: はい
- **説明**: SSH接続先。
- **形式**: `"user@host"` または `"user@host:port"`
- **例**:
  - `"admin@192.168.1.100"`
  - `"robot@myrobot.local"`
  - `"deploy@10.0.0.5:2222"` (非標準ポート使用時)

##### `post_deploy`
- **型**: 文字列（コマンド）
- **必須**: いいえ
- **説明**: デプロイ完了後に実行するコマンド。
- **プレースホルダ**:
  - `{target}`: 設定された `target` の値（例: `"admin@192.168.1.100"`）
  - `{changed_packages}`: 今回のデプロイで更新されたパッケージ名のリスト（スペース区切り）
- **用途例**:
  - Supervisorへの変更通知: `"ssh {target} 'devros supervisor notify --packages {changed_packages}'"`
  - サービスの再起動: `"ssh {target} 'systemctl --user restart robot.service'"`
  - カスタムスクリプト実行: `"ssh {target} '/opt/scripts/post_deploy.sh {changed_packages}'"`

### 使用例

#### 開発環境でのローカルデプロイ

```toml
[deploy.dev]
type = "local"
target_dir = "install_dev"
```

```bash
devros deploy dev
```

#### ステージング環境へのリモートデプロイ

```toml
[deploy.staging]
type = "remote"
method = "rsync"
target_dir = "/opt/staging_ws"
target = "deployer@staging.example.com"
post_deploy = "ssh {target} 'systemctl --user restart staging-robot'"
```

```bash
devros deploy staging
```

#### 本番環境への自動リロード付きデプロイ

```toml
[deploy.production]
type = "remote"
method = "rsync"
target_dir = "/opt/robot_ws"
target = "admin@robot-01.local"
post_deploy = "ssh {target} 'devros supervisor notify --packages {changed_packages}'"
```

```bash
devros deploy production
```

この設定では、デプロイ後に変更されたパッケージがSupervisorに通知され、`watch_packages` に該当するパッケージを含むプロセスが自動的に再起動されます。
