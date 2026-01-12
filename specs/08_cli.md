# CLI仕様 (CLI Specification)

## コマンド体系

### `devros build`
ワークスペース内のパッケージをビルドします。
- オプション:
    - `--package <name>`: 特定のパッケージのみビルド。
    - `--jobs <n>`: 並列ジョブ数指定。
    - `--dry-run`: 実行内容の表示のみ。

### `devros deploy <target>`
指定されたターゲットへデプロイ（実体化・転送）を行います。
- 引数:
    - `target`: `devros.toml` で定義されたデプロイ先名 (例: `local_dev`, `robot_prod`)。

### `devros supervisor <subcommand>`
プロセス管理を行います。
- サブコマンド:
    - `start <group|process>`: プロセス起動。
    - `stop <group|process>`: プロセス停止。
    - `status`: ステータス表示。
    - `tui`: TUIダッシュボード起動。
    - `notify --packages <pkg1> <pkg2> ...`: 指定パッケージの変更を通知し、依存するプロセスを再起動させる。

### `devros watch`
開発ループ（監視 -> ビルド -> デプロイ -> 通知）を自動化します。
- ローカルのファイル変更を監視し、自動的にインクリメンタルビルドとデプロイを実行します。

### `devros env <subcommand>`
環境変数関連のユーティリティです。
- サブコマンド:
    - `shell`: 現在の環境変数を設定するためのシェルスクリプトを出力します。
        - `--shell <type>`: シェルの種類 (bash, zsh, fish)。

### `devros completion`
シェルの入力補完スクリプトを生成します。
- 引数:
    - `--shell <type>`: 生成するシェルの種類 (bash, zsh, fish, powershell, elvish)。
- 機能:
    - サブコマンド、オプションの静的補完。
    - 可能であれば、パッケージ名やターゲット名の動的補完（Dynamic Completion）の提供を目指します。
