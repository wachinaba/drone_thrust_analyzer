#!/bin/bash

# DOI領域選択スクリプト
# 仮想環境をアクティベートしてDOI領域選択ツールを実行

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 仮想環境をアクティベート
echo "仮想環境をアクティベート中..."
source venv/bin/activate

# DOI領域選択ツールを実行
echo "DOI領域選択ツールを起動中..."
echo ""
echo "使用方法:"
echo "  python scripts/select_doi_regions.py                    # 新しい領域を選択"
echo "  python scripts/select_doi_regions.py --load-existing    # 既存の設定を読み込んで編集"
echo "  python scripts/select_doi_regions.py --server-url http://192.168.1.100:5000  # 別のサーバー"
echo ""

# デフォルトで新しい領域選択を実行
python scripts/select_doi_regions.py "$@"
