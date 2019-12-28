# 337. House Robber III

***

**C++** 树+`DFS`

****

思路是这样的：举个简单的例子：`[3,2,3,null,3,null,1]`

**一次递归所做的事：**

1. 包含根节点
   
   * 根值+左子树不包含根节点的情况+右子树不包含根节点的情况
   
2. 不包含根节点

   * max(左子树包含根节点的情况，左子树不包含根节点的情况)+max(右子树包含根节点的情况，右子树不包含根节点的情况)

   

***

代码如下：

```c++
/**
 * Definition for a binary tree node.
 * struct TreeNode {
 *     int val;
 *     TreeNode *left;
 *     TreeNode *right;
 *     TreeNode(int x) : val(x), left(NULL), right(NULL) {}
 * };
 */
class Solution {
public:
    int rob(TreeNode* root) {
        if(!root) return 0;
        pair<int,int> ans=dfs(root);
        return max(ans.first,ans.second);
    }

    pair<int,int> dfs(TreeNode* node)//二叉树的深度优先搜索(其实就是二叉树的先序遍历)
    {
        if(!node) return {0,0};
        pair<int,int> l_p=dfs(node->left);
        pair<int,int> r_p=dfs(node->right);
        return {node->val+l_p.second+r_p.second,max(l_p.first,l_p.second)+max(r_p.first,r_p.second)};
    }
};

```

****

