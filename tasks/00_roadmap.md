# 開発ロードマップ (Roadmap)

## Phase 1: コア、環境管理 & 基本ビルド (Core, Env & Basic Build)
プロジェクトの基盤を構築し、環境変数を適切に管理した上で、ROS 2パッケージ（ament_cmake）を正しくビルドできる状態を目指します。

- [x] **プロジェクト初期化**:
    - [x] `cargo init` (Workspace構成)
    - [x] 依存クレート選定 (`clap`, `serde`, `tokio`, `tracing`, `miette` 等)
    - [x] CIパイプライン構築 (GitHub Actions)
- [x] **設定管理**:
    - [x] `devros.toml` / `devros.local.toml` の定義とパース実装
    - [x] *Test (Unit)*: 設定ファイルのマージロジック、デフォルト値の検証
- [x] **ワークスペース解析**:
    - [x] パッケージ探索 (`package.xml` 検出)
    - [x] `package.xml` パース
    - [x] 依存グラフ構築とトポロジカルソート
    - [x] *Test (Unit)*: グラフ構造の検証、循環依存検出
    - [x] *Test (Integration)*: `tempfile` で作成したダミー階層での探索テスト
- [x] **環境変数管理 (Basic)**:
    - [x] `.dsv` ファイルの生成とパース
    - [x] 環境変数計算ロジック（In-memory）
    - [x] ビルド時の環境変数注入機構
    - [x] `devros env shell` コマンド実装 (確認用)
    - [x] *Test (Unit)*: `.dsv` パース、環境変数計算ロジックの検証
- [x] **ビルド実行 (ament_cmake)**:
    - [x] `cmake` コマンドの発行 (計算された環境変数を適用)
    - [x] 基本的なビルド・インストールフローの実装
    - [ ] `colcon` との互換性検証（最小構成）
    - [x] `install/setup.bash` 生成
    - [ ] *Test (Integration)*: ダミーCMakeパッケージのビルド成功確認
    - [ ] *Test (E2E)*: 実際のROS 2環境でのビルドと `ros2 run` 実行確認

## Phase 2: 高度なビルド機能 (Advanced Build Features)
より多様なパッケージ種別への対応と、ビルドパフォーマンスの最適化を行います。

- [ ] **ビルド機能拡張**:
    - [ ] `ament_python` ビルド対応
    - [ ] **キャッシュシステム**:
        - [ ] Blake3ハッシュ計算 (ソース、設定、依存関係)
        - [ ] キャッシュ判定とスキップロジック
        - [ ] *Test (Unit)*: ファイル変更前後のハッシュ値変化、計算ロジック検証
    - [ ] **Jobserver**:
        - [ ] 全体の並列数制御実装
        - [ ] *Test (Integration)*: 並列実行数の上限が守られているか検証

## Phase 3: デプロイ & Materialization (Deploy)
開発環境から切り離して動作するポータブルなアーティファクト生成を実装します。

- [ ] **Materialization (実体化)**:
    - [ ] `install` ディレクトリのマージ機能
    - [ ] シンボリックリンクの解決（実ファイルコピー）
    - [ ] Pythonパッケージのフリーズ（`.egg-link` 解消）
    - [ ] *Test (Integration)*: シンボリックリンクが実ファイルに置換されているか、Pythonパスが正常か検証
- [ ] **デプロイ実行**:
    - [ ] `devros deploy` コマンド実装
    - [ ] ローカルデプロイ機能
    - [ ] リモートデプロイ機能 (rsync over SSH)
    - [ ] *Test (E2E)*: ローカルデプロイ後のディレクトリでノードが起動できるか確認

## Phase 4: スーパーバイザー (Supervisor)
プロセス管理機能とTUIを実装します。

- [ ] **Daemon基盤**:
    - [ ] Server/Client アーキテクチャ設計
    - [ ] Unix Domain Socket 通信
    - [ ] プロセス起動・監視・停止ロジック
    - [ ] *Test (Unit)*: ステートマシンの遷移テスト
    - [ ] *Test (Integration)*: ダミープロセスの起動・停止・再起動テスト
- [ ] **CLI & TUI**:
    - [ ] `devros supervisor` コマンド群
    - [ ] `ratatui` を用いたダッシュボード実装
- [ ] **リモート管理**:
    - [ ] SSHトンネリングによるリモート接続
    - [ ] `--target` オプション対応
    - [ ] *Test (E2E)*: SSH localhost 接続でのリモート操作テスト

## Phase 5: 開発体験 (DX & Watch)
開発効率を最大化するツール群を実装します。

- [ ] **自動化**:
    - [ ] `devros watch` 実装 (ファイル監視 -> ビルド -> デプロイ -> 通知)
    - [ ] デバウンス処理
    - [ ] *Test (Integration)*: ファイル変更トリガーによるビルド実行の確認
- [ ] **CLI拡張**:
    - [ ] `devros completion` (シェル補完) 実装
- [ ] **品質向上**:
    - [ ] E2Eテスト拡充
    - [ ] ベンチマーク測定 (`colcon` との比較)
