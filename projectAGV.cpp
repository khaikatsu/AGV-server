#include <bits/stdc++.h>
using namespace std;

int n = 5;
int dx[4] = {-1, 1, 0, 0};
int dy[4] = {0, 0, -1, 1};

int main()
{
    int sx, sy, tx, ty;

    cout << "Nhap diem BAT DAU (hang cot tu 0 den 4): ";
    cin >> sx >> sy;

    while (sx < 0 || sx >= n || sy < 0 || sy >= n)
    {
        cout << "Sai! Nhap lai diem BAT DAU: ";
        cin >> sx >> sy;
    }

    cout << "Nhap diem KET THUC (hang cot tu 0 den 4): ";
    cin >> tx >> ty;

    while (tx < 0 || tx >= n || ty < 0 || ty >= n)
    {
        cout << "Sai! Nhap lai diem KET THUC: ";
        cin >> tx >> ty;
    }

    while (sx == tx && sy == ty)
    {
        cout << "Diem ket thuc phai khac diem bat dau. Nhap lai: ";
        cin >> tx >> ty;
    }

    int a[5][5] = {0};

    cout << "\nMa tran ban dau:\n";
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            cout << a[i][j] << " ";
        }
        cout << endl;
    }

    bool visited[5][5] = {false};
    pair<int, int> parent[5][5];

    queue<pair<int, int>> q;
    q.push({sx, sy});
    visited[sx][sy] = true;

    while (!q.empty())
    {
        auto [x, y] = q.front();
        q.pop();

        if (x == tx && y == ty)
            break;

        for (int i = 0; i < 4; i++)
        {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (nx >= 0 && nx < n && ny >= 0 && ny < n && !visited[nx][ny])
            {
                visited[nx][ny] = true;
                parent[nx][ny] = {x, y};
                q.push({nx, ny});
            }
        }
    }

    int result[5][5] = {0};

    int x = tx, y = ty;
    while (!(x == sx && y == sy))
    {
        result[x][y] = 1;
        auto p = parent[x][y];
        x = p.first;
        y = p.second;
    }
    result[sx][sy] = 1;

    cout << "\nMa tran duong di:\n";
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            cout << result[i][j] << " ";
        }
        cout << endl;
    }

    return 0;
}