# スーパーバイザー (Supervisor)

devrosには、ROS 2システムのプロセス管理を行うスーパーバイザー機能が組み込まれています。

## 1. アーキテクチャ

スーパーバイザーは **Daemon (Server)** と **Client** のモデルに基づき動作します。

### Daemon (Headless Server)
- プロセス管理の実体です。バックグラウンドで動作し、プロセスの起動・停止・監視を行います。
- Unix Domain Socket を介して Client からのコマンドを受け付けます。
- **起動方法**:
    - **手動管理**: `devros supervisor daemon start/stop/run` コマンドで明示的に制御。
    - **自動起動**: Client コマンド (`start` 等) 実行時に未起動の場合、自動的にバックグラウンドで起動 (`daemon start` 相当)。

### Client / TUI
- ユーザーインターフェースです。
- CLIコマンドや TUI ダッシュボード (`tui`) はすべて Client として実装され、Daemon に接続して操作を行います。

### リモート接続
SSHトンネリングを利用して、ローカルのCLI/TUIからリモートマシン上で動作するDaemonへ安全に接続できます。
- **仕組み**: SSH接続を介して、リモートのUnix Domain Socketをローカルに転送します。これにより、ローカルの `devros` コマンドがあたかもリモート上で実行されているかのように振る舞います。
- **利点**: リモート側での追加ポート開放や、独自の認証機構構築が不要です。SSHの認証（公開鍵認証など）をそのまま利用できます。

## 2. プロセス管理モデル

設定ファイル (`devros.toml`) で定義されたプロセスグループを管理します。

### 階層構造と環境変数
プロセスはグループに属し、環境変数は継承されます。

- **Group**: 論理的な単位（例: `robot1`）。
    -   グループ全体に共通する環境変数 (`env`) を設定可能。
- **Process**: 実行単位。`ros2 launch` コマンドに対応します。
    -   グループの環境変数を継承し、必要に応じて上書き可能。

## 3. 機能

### 起動・停止・再起動
- グループまたは個別プロセス単位での **起動 (`start`)** ・ **停止 (`stop`)** ・ **再起動 (`restart`)** が可能です。

### 自動リロード (Hot Reload)
`watch_packages` 設定に基づき、パッケージの変更を検知してプロセスを自動再起動します。
これは `notify` コマンドによってトリガーされます。

**フロー:**
1.  デプロイ完了後、外部から `devros supervisor notify --packages <pkg_names...>` が実行される。
2.  Daemon は、通知されたパッケージに依存する（`watch_packages` に含まれる）稼働中のプロセスを特定。
3.  対象プロセスに対し、Graceful Stop -> Start (再起動) を自動的に実行。

## 4. インターフェース

### TUI (Text User Interface)
`ratatui` を使用したダッシュボード。Daemonに接続して状態を表示・操作します。
- **Process List**: プロセス一覧、状態表示。
- **Log View**: ログストリーム確認。
- **Status Bar**: 全体のステータス表示。

### CLI コマンド
IPC経由でデーモンを操作します。

#### 共通オプション (リモート接続)
`daemon` 以外の全てのサブコマンドで、以下のオプションを使用してリモートのSupervisorを操作できます。

- `--host <connection_string>`: SSH接続先を直接指定します（例: `admin@192.168.1.100`）。
- `--target <deploy_target_name>`: `devros.toml` の `[deploy.<target>]` セクションで定義されたターゲット名を指定します。そのセクションの `target` 設定値（接続先）が使用されます。

**使用例:**
```bash
# 直接ホスト指定でステータス確認
devros supervisor status --host admin@robot.local

# 定義済みターゲット(robot_prod)に接続してTUIを起動
devros supervisor tui --target robot_prod

# リモートのプロセスを再起動
devros supervisor restart robot1.perception --target robot_prod
```

#### デーモン管理
- `daemon run`: Daemonを起動（フォアグラウンド）。Systemd等での利用を想定。
- `daemon start`: Daemonを起動（バックグラウンド）。
- `daemon stop`: Daemonを停止。

#### プロセス管理
- `start <target>`: プロセス起動 (Targetは `group` または `group.process`)。
- `stop <target>`: プロセス停止。
- `restart <target>`: プロセス再起動。
- `notify --packages <pkgs>`: 変更通知（自動リロードのトリガー）。
- `status`: ステータス表示。

## 5. プロセス管理設定 (devros.toml)

Supervisorで管理するプロセスは、`devros.toml` の `[supervisor.<group>.<process>]` セクションで定義します。

### 基本構成

```toml
# グループレベルの設定（オプション）
[supervisor.robot1]
env = { "ROS_DOMAIN_ID" = "10", "RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" }

# プロセス設定
[supervisor.robot1.perception]
type = "launch"
package = "perception_bringup"
file = "perception.launch.py"
watch_packages = ["perception_nodes", "camera_driver"]
env = { "LOG_LEVEL" = "debug" }

[supervisor.robot1.navigation]
type = "launch"
package = "nav2_bringup"
file = "navigation_launch.py"
watch_packages = ["nav2_nodes"]
```

### 設定項目詳細

#### グループ設定 `[supervisor.<group>]`

グループレベルの設定は任意です。設定した場合、グループ内の全プロセスに適用されます。

##### `env`
- **型**: テーブル（キー=値のマップ）
- **必須**: いいえ
- **説明**: グループ内の全プロセスに適用される環境変数。
- **継承**: プロセス個別の `env` 設定によって上書き可能。
- **例**:
  ```toml
  [supervisor.robot1]
  env = { "ROS_DOMAIN_ID" = "10", "RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" }
  ```

#### プロセス設定 `[supervisor.<group>.<process>]`

##### `type`
- **型**: 文字列
- **必須**: はい
- **説明**: プロセスのタイプ。
- **有効な値**:
  - `"launch"`: `ros2 launch` コマンドを実行

##### `package`
- **型**: 文字列
- **必須**: はい（`type = "launch"` の場合）
- **説明**: launchファイルを含むROS 2パッケージ名。

##### `file`
- **型**: 文字列
- **必須**: はい（`type = "launch"` の場合）
- **説明**: 実行するlaunchファイル名。
- **例**: `"perception.launch.py"`, `"navigation.launch.xml"`

##### `watch_packages`
- **型**: 文字列配列（パッケージ名）
- **必須**: いいえ（デフォルト: `[]`）
- **説明**: このプロセスが依存するパッケージのリスト。これらのパッケージに変更があり、`devros supervisor notify --packages <pkg>` で通知された場合、このプロセスは自動的に再起動されます。
- **用途**: Hot Reload機能の実現。開発中のパッケージを指定することで、コード変更時の自動再起動が可能になります。
- **例**:
  ```toml
  watch_packages = ["perception_nodes", "camera_driver", "image_processor"]
  ```

##### `env`
- **型**: テーブル（キー=値のマップ）
- **必須**: いいえ
- **説明**: プロセス固有の環境変数。
- **継承**: グループレベルの `env` を継承し、同名のキーは上書きします。
- **例**:
  ```toml
  env = { "LOG_LEVEL" = "debug", "CAMERA_DEVICE" = "/dev/video0" }
  ```

### 環境変数の継承ルール

環境変数は以下の優先順位でマージされます（後者ほど優先度が高い）：

1. システム環境変数
2. ワークスペースの環境設定（`setup.bash` 等）
3. グループレベルの `env`
4. プロセスレベルの `env`

#### 例

```toml
[supervisor.robot1]
env = { "ROS_DOMAIN_ID" = "10", "LOG_LEVEL" = "info" }

[supervisor.robot1.perception]
env = { "LOG_LEVEL" = "debug", "CAMERA_FPS" = "30" }
```

`perception` プロセスの実際の環境変数：
- `ROS_DOMAIN_ID = "10"` （グループから継承）
- `LOG_LEVEL = "debug"` （プロセス設定で上書き）
- `CAMERA_FPS = "30"` （プロセス固有）

### 設定例

#### シンプルな構成

```toml
[supervisor.default.my_node]
type = "launch"
package = "my_package"
file = "my_node.launch.py"
```

#### 複数ロボットの管理

```toml
# ロボット1
[supervisor.robot1]
env = { "ROS_DOMAIN_ID" = "10" }

[supervisor.robot1.perception]
type = "launch"
package = "perception_bringup"
file = "perception.launch.py"
watch_packages = ["perception_nodes"]

[supervisor.robot1.navigation]
type = "launch"
package = "nav2_bringup"
file = "navigation.launch.py"
watch_packages = ["nav2_nodes"]

# ロボット2
[supervisor.robot2]
env = { "ROS_DOMAIN_ID" = "20" }

[supervisor.robot2.perception]
type = "launch"
package = "perception_bringup"
file = "perception.launch.py"
watch_packages = ["perception_nodes"]

[supervisor.robot2.navigation]
type = "launch"
package = "nav2_bringup"
file = "navigation.launch.py"
watch_packages = ["nav2_nodes"]
```

#### 開発環境での自動リロード設定

```toml
[supervisor.dev]
env = { "ROS_DOMAIN_ID" = "0", "RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" }

[supervisor.dev.perception]
type = "launch"
package = "perception_bringup"
file = "perception.launch.py"
# 開発中のパッケージを全て列挙
watch_packages = [
    "perception_nodes",
    "camera_driver",
    "image_processor",
    "object_detector"
]
env = { "LOG_LEVEL" = "debug" }

[supervisor.dev.planning]
type = "launch"
package = "planning_bringup"
file = "planning.launch.py"
watch_packages = ["planning_nodes", "path_planner"]
env = { "LOG_LEVEL" = "debug" }
```

この設定では、`devros watch --deploy robot_dev` を実行すると、コード変更時に自動的にビルド→デプロイ→プロセス再起動が行われます。
